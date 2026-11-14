<script setup lang="ts">
import { computed, ref } from 'vue'
import type { TeachPoint } from '../api/following'
const props = defineProps<{reference:TeachPoint[];raw:TeachPoint[];track:TeachPoint[];pose:TeachPoint|null}>()
const zoom=ref(1), panX=ref(0), panY=ref(0)
const showRaw=ref(true), showTrack=ref(true), showReference=ref(true)
const bounds=computed(()=>{
  const points=[...props.reference,...props.track,...props.raw,...(props.pose?[props.pose]:[])]
  if(!points.length)return {x:0,y:0,scale:1}
  let minX=Infinity,maxX=-Infinity,minY=Infinity,maxY=-Infinity
  for(const p of points){minX=Math.min(minX,p.x);maxX=Math.max(maxX,p.x);minY=Math.min(minY,p.y);maxY=Math.max(maxY,p.y)}
  return {x:(minX+maxX)/2,y:(minY+maxY)/2,scale:Math.min(720/Math.max(2,maxX-minX),270/Math.max(2,maxY-minY))}
})
function xy(p:TeachPoint){return {x:400+(p.x-bounds.value.x)*bounds.value.scale,y:170-(p.y-bounds.value.y)*bounds.value.scale}}
function line(points:TeachPoint[]){return points.map(p=>{const q=xy(p);return `${q.x},${q.y}`}).join(' ')}
const viewBox=computed(()=>`${panX.value} ${panY.value} ${800/zoom.value} ${340/zoom.value}`)
function reset(){zoom.value=1;panX.value=panY.value=0}
function scale(f:number){zoom.value=Math.max(1,Math.min(12,zoom.value*f));if(zoom.value===1)reset()}
let drag:{x:number;y:number;px:number;py:number}|null=null
function down(e:PointerEvent){(e.currentTarget as SVGSVGElement).setPointerCapture(e.pointerId);drag={x:e.clientX,y:e.clientY,px:panX.value,py:panY.value}}
function move(e:PointerEvent){if(!drag)return;const m=(e.currentTarget as SVGSVGElement).getScreenCTM();if(m) {panX.value=drag.px-(e.clientX-drag.x)/m.a;panY.value=drag.py-(e.clientY-drag.y)/m.d}}
</script>
<template>
  <div class="map-tools"><el-checkbox v-model="showReference">参考路径</el-checkbox><el-checkbox v-model="showRaw">录制轨迹</el-checkbox><el-checkbox v-model="showTrack">实际轨迹</el-checkbox>
    <el-button size="small" @click="scale(1.3)">放大</el-button><el-button size="small" @click="scale(1/1.3)">缩小</el-button><el-button size="small" @click="reset">复位</el-button></div>
  <svg class="following-map" :viewBox="viewBox" role="img" aria-label="map 固定原点下的示教参考路径和实际轨迹"
    @pointerdown="down" @pointermove="move" @pointerup="drag=null" @pointercancel="drag=null" @wheel.prevent="scale($event.deltaY<0?1.1:1/1.1)">
    <polyline v-if="showRaw" :points="line(raw)" fill="none" stroke="#b2a1ce" stroke-width="2" vector-effect="non-scaling-stroke"/>
    <polyline v-if="showReference" :points="line(reference)" fill="none" stroke="#2682dc" stroke-width="2.5" vector-effect="non-scaling-stroke"/>
    <polyline v-if="showTrack" :points="line(track)" fill="none" stroke="#ec9a28" stroke-width="2.5" vector-effect="non-scaling-stroke"/>
    <template v-if="reference.length && showReference">
      <circle :cx="xy(reference[0]!).x" :cy="xy(reference[0]!).y" r="5" fill="#23965c"/><text :x="xy(reference[0]!).x+8" :y="xy(reference[0]!).y-8" font-size="13" fill="#167a43">起点</text>
      <circle :cx="xy(reference[reference.length-1]!).x" :cy="xy(reference[reference.length-1]!).y" r="5" fill="#df4848"/><text :x="xy(reference[reference.length-1]!).x+8" :y="xy(reference[reference.length-1]!).y+18" font-size="13" fill="#ba3030">终点</text>
    </template>
    <g v-if="pose" :transform="`translate(${xy(pose).x},${xy(pose).y}) rotate(${-((pose.yaw||0)*180/Math.PI)})`"><path d="M 11 0 L -6 -6 L -3 0 L -6 6 Z" fill="#103c53"/></g>
    <text v-if="!reference.length && !raw.length" x="400" y="150" text-anchor="middle" fill="#7a8e84">选择已保存路径，或开始手柄示教录制</text>
  </svg>
  <div class="legend">蓝：参考路径　紫：本次录制　橙：实际跟随　绿：起点　红：终点 · map 坐标，米；等比例显示，可拖动</div>
</template>
<style scoped>
.following-map{width:100%;height:clamp(240px,34vh,360px);background:#f5f8f6;border:1px solid #e0e8e2;border-radius:8px;touch-action:none;cursor:grab}.map-tools{display:flex;align-items:center;flex-wrap:wrap;gap:8px;margin-bottom:10px}.map-tools .el-button{margin:0}.legend{font-size:12px;color:#718178;margin-top:8px}
</style>
