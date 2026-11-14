<script setup lang="ts">
import { computed, onMounted, onUnmounted, ref } from 'vue'
import { Bell, User, ArrowDown } from '@element-plus/icons-vue'
import { useRoute, useRouter } from 'vue-router'
import EmergencyControls from './EmergencyControls.vue'
import RuntimeDiagnostics from './RuntimeDiagnostics.vue'

const route = useRoute()
const router = useRouter()
function navigate(target: unknown) { void router.push(String(target)) }
const currentTime = ref(new Date())
let timer: ReturnType<typeof setInterval> | undefined
const pageTitle = computed(() => String(route.meta.title ?? '系统总览'))
const formattedTime = computed(() => currentTime.value.toLocaleString('zh-CN', {
  year: 'numeric', month: '2-digit', day: '2-digit',
  hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false,
}))
onMounted(() => { timer = setInterval(() => { currentTime.value = new Date() }, 1000) })
onUnmounted(() => { if (timer) clearInterval(timer) })
</script>

<template>
  <el-header class="topbar">
    <!-- 左侧：页面标题 -->
    <div class="breadcrumb">
      <span class="breadcrumb-muted">山东大学农业机器人</span>
      <span class="breadcrumb-separator">/</span>
      <span class="breadcrumb-current">{{ pageTitle }}</span>
    </div>

    <RuntimeDiagnostics />
    <EmergencyControls />

    <!-- 右侧：系统时间与用户信息 -->
    <div class="topbar-right">
      <div class="topbar-time">
        {{ formattedTime }}
      </div>

      <el-button circle text aria-label="运行记录" @click="router.push('/settings')">
        <el-icon :size="19">
          <Bell />
        </el-icon>
      </el-button>

      <el-dropdown @command="navigate">
        <div class="user-entry">
          <el-avatar :size="32" class="user-avatar">
            <el-icon>
              <User />
            </el-icon>
          </el-avatar>

          <span>本地控制台</span>

          <el-icon>
            <ArrowDown />
          </el-icon>
        </div>

        <template #dropdown>
          <el-dropdown-menu>
            <el-dropdown-item command="/devices">设备与安全</el-dropdown-item>
            <el-dropdown-item command="/settings">运行记录</el-dropdown-item>
          </el-dropdown-menu>
        </template>
      </el-dropdown>
    </div>
  </el-header>
</template>

<style scoped>
/* ============================================================
   顶部导航栏
   ============================================================ */

.topbar {
  position: sticky;
  z-index: 5;
  top: 0;

  display: flex;
  align-items: center;
  justify-content: space-between;

  height: 66px;
  padding: 0 24px;

  border-bottom: 1px solid #e9edf3;
  background: rgb(255 255 255 / 96%);
}

/* ============================================================
   左侧：面包屑导航
   ============================================================ */

.breadcrumb {
  display: flex;
  align-items: center;
  gap: 12px;

  flex-shrink: 0;
  font-size: 13px;
}

.breadcrumb-muted {
  color: #8896a7;
}

.breadcrumb-separator {
  color: #c3cbd5;
}

.breadcrumb-current {
  color: #273747;
  font-weight: 600;
}

/* ============================================================
   急停控制：靠右排列，紧挨着时间
   ============================================================ */

.emergency-controls {
  display: flex;
  align-items: center;
  justify-content: flex-end;
  gap: 10px;

  margin: 0 16px 0 12px;

  flex-shrink: 0;
  white-space: nowrap;
}

.emergency-status {
  display: flex;
  align-items: center;
  gap: 7px;

  margin-right: 4px;

  font-size: 12px;
  font-weight: 600;
}

.status-dot {
  width: 8px;
  height: 8px;

  flex-shrink: 0;
  border-radius: 50%;
}

.status-stop {
  color: var(--el-color-danger);
}

.status-stop .status-dot {
  background: var(--el-color-danger);
}

.status-ready {
  color: var(--el-color-success);
}

.status-ready .status-dot {
  background: var(--el-color-success);
}

.emergency-button,
.reset-button {
  margin-left: 0;
  font-weight: 600;
}

/* ============================================================
   右侧：系统时间与用户信息
   ============================================================ */

.topbar-right {
  display: flex;
  align-items: center;
  gap: 14px;

  flex-shrink: 0;
}

.topbar-time {
  color: #748398;

  font-size: 12px;
  font-variant-numeric: tabular-nums;

  white-space: nowrap;
}

.user-entry {
  display: flex;
  align-items: center;
  gap: 9px;

  color: #415165;
  font-size: 12px;

  cursor: pointer;
  white-space: nowrap;
}

.user-avatar {
  background: #e4f3ea;
  color: #25845c;
}

/* ============================================================
   响应式布局
   ============================================================ */

@media (max-width: 1200px) {
  .topbar {
    padding: 0 16px;
  }

  .topbar-time {
    display: none;
  }

  .emergency-controls {
    margin: 0 12px 0 8px;
    gap: 6px;
  }
}

@media (max-width: 900px) {
  .emergency-status {
    display: none;
  }

  .topbar-right {
    gap: 8px;
  }
}

@media (max-width: 760px) {
  .topbar {
    padding: 8px 10px;
    height: auto;
    min-height: 66px;
    box-sizing: border-box;
    gap: 6px;
  }

  .breadcrumb, .topbar-right { display: none; }

  .breadcrumb {
    gap: 6px;
    font-size: 11px;
  }

  .emergency-controls {
    margin: 0 0 0 5px;
    gap: 5px;
    flex: 1;
    min-width: 0;
  }

  .emergency-button,
  .reset-button {
    padding: 7px 9px;
    font-size: 11px;
  }

  .user-entry > span,
  .user-entry > .el-icon {
    display: none;
  }

  .topbar-right {
    gap: 5px;
  }
}
</style>
