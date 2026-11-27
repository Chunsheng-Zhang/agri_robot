// Offline behavioral tests of the actual Vue scripts; no HTTP, ROS or vehicle control.
const assert = require('node:assert/strict')
const { test } = require('node:test')
const fs = require('node:fs')
const path = require('node:path')
const root = path.resolve(__dirname, '..')
const deps = [path.join(root, 'agri_web')]
const ts = require(require.resolve('typescript', { paths: deps }))
const vue = require(require.resolve('vue', { paths: deps }))
const { parse } = require(require.resolve('@vue/compiler-sfc', { paths: deps }))

function fixture(kind) {
  const props = vue.reactive({ kind, modes: ['base', kind] })
  const devices = vue.ref({
    safety: { mode: kind, owner: '', motion_inhibited: false },
    waypoint: { state: 'IDLE', count: 5, index: 0 },
    tracking: { state: 'IDLE', length: 10, progress: 0 },
    planning: { state: 'IDLE' },
  })
  const context = vue.ref({ active_run: null, missions: [], runs: [] })
  const safetyFresh = vue.ref(true), calls = [], refreshes = [], messages = [], scope = vue.effectScope()
  const modules = {
    vue,
    'element-plus': { ElMessage: { success: message => messages.push(['success', message]), error: message => messages.push(['error', message]) }, ElMessageBox: { confirm: async () => { calls.push(['confirm']) } } },
    '../../composables/useRealtime': { useRealtime: () => ({ devices, safetyFresh, isFresh: status => !!status && !status.stale, refresh: async () => {} }) },
    '../../composables/useWorkspace': { useWorkspace: () => ({ context, refresh: async () => { refreshes.push(1) }, error: vue.ref(''), loading: vue.ref(false) }) },
    '../../api/workspace': {
      taskAction: async (...args) => { calls.push(['task', ...args]) },
      cancelPlanning: async () => { calls.push(['cancel_plan']) },
      selectMode: async mode => { calls.push(['mode', mode]) },
      deleteRun: async id => { calls.push(['delete', id]); return { success: true, data_preserved: true } },
      modeLabels: { base: '基础待命', waypoint: '多点导航', coverage: '全覆盖规划与跟踪', following: '路径跟随' },
      workModes: ['base','waypoint','coverage','following'], workModeText: mode => mode,
    },
  }
  for (const name of ['useRealtime', 'useWorkspace']) modules[`../composables/${name}`] = modules[`../../composables/${name}`]
  modules['../api/workspace'] = modules['../../api/workspace']
  function load(file, names) {
    const { descriptor } = parse(fs.readFileSync(path.join(root, 'agri_web/src', file), 'utf8'))
    const code = ts.transpileModule(`${descriptor.scriptSetup.content}\nreturn { ${names.join(', ')} }`, {
      compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2022 },
    }).outputText
    return scope.run(() => new Function('require', 'exports', 'defineProps', 'withDefaults', code)(
      name => { if (name.endsWith('.vue')) return {}; assert.ok(modules[name], `Unexpected import: ${name}`); return modules[name] },
      {}, () => props, value => value))
  }
  return { props, devices, context, safetyFresh, calls, refreshes, messages, modules, load, stop: () => scope.stop() }
}
const controls = f => f.load('components/tasks/TaskControls.vue', [
  'source', 'activeRun', 'state', 'displayState', 'planning', 'coveragePhase', 'progress', 'planningProgress', 'actualControllerLabel',
  'canPause', 'canResume', 'canCancel', 'operate',
])
test('idle or completed default/previous PP is not shown as the actual algorithm; fresh running LQR is',()=>{
  const f=fixture('coverage'),c=controls(f)
  f.devices.value.tracking.controller_label='PP'
  for(const state of ['IDLE','READY','COMPLETED','CANCELED','FAULT','ACQUIRING']){
    f.devices.value.tracking.state=state
    assert.equal(c.actualControllerLabel.value,undefined)
  }
  f.devices.value.tracking.controller_label='LQR'
  for(const state of ['RUNNING','PAUSED']){
    f.devices.value.tracking.state=state
    assert.equal(c.actualControllerLabel.value,'LQR')
  }
  f.devices.value.tracking.stale=true
  assert.equal(c.actualControllerLabel.value,undefined)
  assert.equal(f.calls.length,0)
  f.stop()
})

test('waypoint section cannot act on a running coverage task when global mode is coverage', async () => {
  const f = fixture('waypoint')
  f.devices.value.safety.mode = 'coverage'
  f.devices.value.tracking.state = 'RUNNING'
  f.devices.value.planning.state = 'PLANNING'
  f.context.value.active_run = { mode: 'coverage', name: 'Coverage', stage: 'planning', state: 'PLANNING' }
  const c = controls(f)
  assert.equal(c.source.value, 'waypoint')
  assert.equal(c.activeRun.value, undefined)
  assert.equal(c.state.value, 'IDLE')
  assert.equal(c.displayState.value, 'IDLE')
  assert.equal(c.planning.value, false)
  assert.equal(c.coveragePhase.value, undefined)
  for (const action of ['pause', 'resume', 'cancel', 'cancel_plan']) await c.operate(action)
  assert.deepEqual(f.calls, [])
  f.stop()
})
test('waypoint pause/resume/cancel target waypoint even if another mode is selected', async () => {
  const f = fixture('waypoint'), c = controls(f)
  f.devices.value.waypoint.state = 'RUNNING'
  await c.operate('pause')
  f.devices.value.waypoint.state = 'PAUSED'
  f.devices.value.safety.mode = 'coverage'
  await c.operate('resume') // A paused task cannot resume in the other mode.
  f.devices.value.safety.mode = 'waypoint'
  await c.operate('resume')
  await c.operate('cancel')
  assert.deepEqual(f.calls, [['task', 'waypoint', 'pause'], ['task', 'waypoint', 'resume'], ['task', 'waypoint', 'cancel']])
  f.stop()
})
test('coverage approach and tracking share their own controls and never route to waypoint', async () => {
  const f = fixture('coverage'), c = controls(f)
  f.context.value.active_run = { mode: 'coverage', name: 'Coverage', stage: 'tracking', state: 'RUNNING' }
  f.devices.value.tracking = { state: 'RUNNING', phase: 'APPROACHING', length: 10, progress: 0 }
  assert.equal(c.activeRun.value.name, 'Coverage')
  assert.equal(c.coveragePhase.value, 'APPROACHING')
  await c.operate('pause')
  f.devices.value.tracking.state = 'PAUSED'
  await c.operate('resume')
  f.devices.value.tracking.phase = 'ALIGNING'
  assert.equal(c.coveragePhase.value, 'ALIGNING')
  f.devices.value.tracking.phase = 'TRACKING'
  f.devices.value.tracking.progress = 5
  assert.equal(c.coveragePhase.value, 'TRACKING')
  assert.equal(c.progress.value, 50)
  await c.operate('cancel')
  assert.deepEqual(f.calls, [['task', 'tracking', 'pause'], ['task', 'tracking', 'resume'], ['task', 'tracking', 'cancel']])
  f.stop()
})
test('resume and other actions remain disabled for stale status or unsafe state', async () => {
  const f = fixture('coverage'), c = controls(f)
  f.devices.value.tracking.state = 'PAUSED'
  f.safetyFresh.value = false
  await c.operate('resume')
  f.safetyFresh.value = true
  f.devices.value.safety.motion_inhibited = true
  await c.operate('resume')
  f.devices.value.safety.motion_inhibited = false
  f.devices.value.tracking.stale = true
  assert.equal(c.canCancel.value, false)
  await c.operate('cancel')
  await c.operate('resume')
  assert.deepEqual(f.calls, [])
  f.stop()
})
test('only coverage page can cancel fresh planning, without cancelling a controller', async () => {
  const f = fixture('coverage'), c = controls(f)
  f.devices.value.planning.state = 'PLANNING'
  assert.equal(c.displayState.value, 'PLANNING')
  await c.operate('cancel_plan')
  f.devices.value.planning.stale = true
  await c.operate('cancel_plan')
  assert.deepEqual(f.calls, [['cancel_plan']])
  f.stop()
})
test('mission lists and run histories follow the fixed page category', async () => {
  const f = fixture('waypoint')
  f.context.value.missions = [{ id: 'w', mode: 'waypoint' }, { id: 'c', mode: 'coverage' }]
  f.context.value.runs = [{ id: 'wr', mode: 'waypoint' }, { id: 'cr', mode: 'coverage' }]
  const workspace = f.load('components/tasks/TaskWorkspace.vue', ['missions', 'selected', 'selectedId', 'saved'])
  const history = f.load('components/tasks/TaskRunHistory.vue', ['runs', 'inProgress'])
  assert.deepEqual(workspace.missions.value.map(m => m.id), ['w'])
  assert.deepEqual(history.runs.value.map(r => r.id), ['wr'])
  workspace.saved({ id: 'c', mode: 'coverage' })
  assert.equal(workspace.selected.value, null)
  workspace.saved({ id: 'w', mode: 'waypoint' })
  assert.equal(workspace.selected.value.id, 'w')
  f.props.kind = 'coverage'
  await vue.nextTick()
  assert.equal(workspace.selectedId.value, undefined)
  assert.deepEqual(workspace.missions.value.map(m => m.id), ['c'])
  assert.deepEqual(history.runs.value.map(r => r.id), ['cr'])
  assert.equal(history.inProgress('PAUSED'), true)
  assert.equal(history.inProgress('COMPLETED'), false)
  assert.deepEqual(f.calls, []) // Category changes never select a robot mode.
  f.stop()
})
test('page-scoped mode selection ignores modes outside its own buttons', async () => {
  const f = fixture('waypoint')
  f.devices.value.safety.mode = 'base'
  const modes = f.load('components/ModeSelector.vue', ['availableModes', 'change'])
  assert.deepEqual(modes.availableModes.value.map(m => m.mode), ['base', 'waypoint'])
  assert.deepEqual(f.calls, []) // Loading a page does not mutate the mode.
  await modes.change('coverage')
  assert.deepEqual(f.calls, [])
  await modes.change('waypoint')
  assert.deepEqual(f.calls, [['mode', 'waypoint']])
  f.stop()
})
test('explicit mode switch still requires confirmation when another task is running', async () => {
  const f = fixture('coverage')
  f.devices.value.safety.mode = 'waypoint'
  f.context.value.active_run = { mode: 'waypoint', state: 'RUNNING' }
  const modes = f.load('components/ModeSelector.vue', ['change'])
  await modes.change('coverage')
  assert.deepEqual(f.calls, [['confirm'], ['mode', 'coverage']])
  f.stop()
})

function deletion(f, state = 'COMPLETED') {
  f.props.run = { id: 'run-a', name: 'Test', state, mission_revision: 1, created_at: '2026-10-30T10:00:00+08:00' }
  return f.load('components/tasks/RunDeleteButton.vue', ['canDelete', 'busy', 'remove'])
}
test('record deletion requires confirmation and refreshes only after successful deletion', async () => {
  const f = fixture('waypoint'), button = deletion(f)
  await button.remove()
  assert.deepEqual(f.calls, [['confirm'], ['delete', 'run-a']])
  assert.equal(f.refreshes.length, 1)
  assert.match(f.messages[0][1], /原始实验数据已保留/)
  assert.equal(button.busy.value, false)
  f.stop()
})
test('cancelled confirmation does not delete anything', async () => {
  const f = fixture('waypoint'), button = deletion(f)
  f.modules['element-plus'].ElMessageBox.confirm = async () => { throw Error('cancel') }
  await button.remove()
  assert.deepEqual(f.calls, [])
  assert.deepEqual(f.messages, [])
  assert.equal(f.refreshes.length, 0)
  assert.equal(button.busy.value, false)
  f.stop()
})
test('unfinished or unknown record states never open deletion confirmation', async () => {
  const f = fixture('coverage'), button = deletion(f)
  for (const state of ['PREPARING', 'PLANNING', 'ACQUIRING', 'RUNNING', 'PAUSED', 'WAITING_PATH', 'UNKNOWN']) {
    f.props.run.state = state
    assert.equal(button.canDelete.value, false)
    await button.remove()
  }
  assert.deepEqual(f.calls, [])
  f.stop()
})
test('task starting or row changing while confirmation is open invalidates deletion', async () => {
  const f = fixture('coverage'), button = deletion(f, 'READY')
  f.modules['element-plus'].ElMessageBox.confirm = async () => { f.props.run.state = 'RUNNING' }
  await button.remove()
  f.props.run.state = 'COMPLETED'
  f.modules['element-plus'].ElMessageBox.confirm = async () => { f.props.run = { ...f.props.run, id: 'run-b' } }
  await button.remove()
  assert.deepEqual(f.calls, [])
  f.stop()
})
test('duplicate clicks do not issue two deletes and backend refusal is shown without a success message', async () => {
  const f = fixture('waypoint'), button = deletion(f)
  let finish
  f.modules['element-plus'].ElMessageBox.confirm = () => new Promise(resolve => { finish = resolve })
  f.modules['../../api/workspace'].deleteRun = async id => { f.calls.push(['delete', id]); throw Error('控制器仍在使用这条记录') }
  const pending = button.remove()
  assert.equal(button.busy.value, true)
  await button.remove()
  finish()
  await pending
  assert.deepEqual(f.calls, [['delete', 'run-a']])
  assert.deepEqual(f.messages, [['error', '控制器仍在使用这条记录']])
  assert.equal(button.busy.value, false)
  assert.equal(f.refreshes.length, 0)
  f.stop()
})

test('planning progress follows fresh ROS feedback and never leaks into waypoint view',()=>{
  const f=fixture('coverage'), c=controls(f)
  f.devices.value.planning={state:'PLANNING',search_progress:{phase:'REFINE',stage_completed:3,stage_total:6,evaluated_count:16,feasible_count:14,elapsed_seconds:50}}
  assert.equal(c.planningProgress.value.phase,'REFINE')
  assert.equal(c.planningProgress.value.evaluated_count,16)
  f.devices.value.planning.stale=true
  assert.equal(c.planningProgress.value,null)
  f.devices.value.planning.stale=false;f.props.kind='waypoint'
  assert.equal(c.planningProgress.value,null)
  f.props.kind='coverage';f.devices.value.planning.state='CANCELED'
  assert.equal(c.planningProgress.value,null)
  f.stop()
})
