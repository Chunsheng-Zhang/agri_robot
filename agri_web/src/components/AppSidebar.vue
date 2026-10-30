<script setup lang="ts">
import { computed } from 'vue'
import { useRealtime } from '../composables/useRealtime'
import {
  Monitor,
  MapLocation,
  List,
  Guide,
  Cpu,
  Setting,
  Connection,
} from '@element-plus/icons-vue'

const { robot, robotFresh, poseFresh, error } = useRealtime()
const bridgeConnected = computed(() => robotFresh.value && robot.value?.ros_connected === true)

const menuItems = [
  { path: '/', title: '系统总览', icon: Monitor },
  { path: '/monitor', title: '作业监控', icon: MapLocation },
  { path: '/tasks', title: '多点导航', icon: List },
  { path: '/planning', title: '全覆盖规划与跟踪', icon: Guide },
  { path: '/devices', title: '设备管理', icon: Cpu },
  { path: '/collaboration', title: '协同作业', icon: Connection },
  { path: '/settings', title: '系统设置', icon: Setting },
]
</script>

<template>
  <el-aside width="228px" class="sidebar">
    <div class="brand">
      <div class="brand-logo">
        <el-icon :size="27"><MapLocation /></el-icon>
      </div>

      <div>
        <div class="brand-title">AGRI ROBOT</div>
        <div class="brand-subtitle">农业机器人管理平台</div>
      </div>
    </div>

    <div class="menu-caption">系统导航</div>

    <el-menu
      router
      :default-active="$route.path"
      class="side-menu"
      background-color="transparent"
      text-color="#a8b8cc"
      active-text-color="#ffffff"
    >
      <el-menu-item
        v-for="item in menuItems"
        :key="item.path"
        :index="item.path"
      >
        <el-icon><component :is="item.icon" /></el-icon>
        <span>{{ item.title }}</span>
      </el-menu-item>
    </el-menu>

    <div class="sidebar-bottom">
      <div class="connection-card">
        <div class="connection-heading">
          <el-icon><Connection /></el-icon>
          <span>通信状态</span>
        </div>

        <div class="connection-status">
          <span class="status-dot" :class="{ disconnected: !poseFresh }"></span>
          {{ !robotFresh ? '后端状态未更新' : poseFresh ? '机器人位姿在线' : bridgeConnected ? 'ROS 桥接在线' : 'ROS 桥接未连接' }}
        </div>

        <div class="connection-note">{{ error || (poseFresh ? '正在显示实时数据' : '等待有效定位与设备数据') }}</div>
      </div>

      <div class="sidebar-version">AGRI ROBOT · WEB v0.1.0</div>
    </div>
  </el-aside>
</template>

<style scoped>
.sidebar {
  position: fixed;
  z-index: 10;
  inset: 0 auto 0 0;
  display: flex;
  flex-direction: column;
  overflow-y: auto;
  background: #172b3c;
  color: white;
}

.brand {
  display: flex;
  align-items: center;
  gap: 12px;
  height: 84px;
  padding: 0 20px;
  border-bottom: 1px solid rgb(255 255 255 / 8%);
}

.brand-logo {
  display: flex;
  align-items: center;
  justify-content: center;
  width: 44px;
  height: 44px;
  flex-shrink: 0;
  border-radius: 12px;
  background: #25845c;
}

.brand-title {
  font-size: 16px;
  font-weight: 750;
  letter-spacing: 0.8px;
}

.brand-subtitle {
  margin-top: 5px;
  color: #a9b9c9;
  font-size: 11px;
}

.menu-caption {
  padding: 27px 22px 12px;
  color: #8094a8;
  font-size: 11px;
  letter-spacing: 1.4px;
}

.side-menu {
  flex: 1;
  padding: 0 12px;
  border-right: none;
}

.side-menu .el-menu-item {
  height: 48px;
  margin-bottom: 5px;
  border-radius: 7px;
}

.side-menu .el-menu-item.is-active {
  background: #25845c !important;
  font-weight: 600;
}

.side-menu .el-menu-item:hover {
  background: rgb(255 255 255 / 8%);
}

.sidebar-bottom {
  padding: 16px;
}

.connection-card {
  padding: 15px;
  border: 1px solid rgb(255 255 255 / 9%);
  border-radius: 9px;
  background: rgb(255 255 255 / 4%);
}

.connection-heading {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-bottom: 13px;
  color: #dce6ef;
  font-size: 12px;
}

.connection-status {
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 12px;
}

.status-dot {
  display: inline-block;
  width: 7px;
  height: 7px;
  border-radius: 50%;
  background: #35bd82;
}

.status-dot.disconnected { background: #dba956; }

.connection-note {
  margin-top: 9px;
  color: #91a4b7;
  font-size: 11px;
}

.sidebar-version {
  margin-top: 19px;
  color: #72869a;
  font-size: 10px;
  text-align: center;
}

@media (max-width: 760px) {
  .sidebar {
    width: 66px !important;
  }

  .brand {
    justify-content: center;
    padding: 0;
  }

  .brand > div:last-child,
  .menu-caption,
  .side-menu .el-menu-item span,
  .sidebar-bottom {
    display: none;
  }

  .side-menu {
    padding: 0 6px;
  }

  .side-menu .el-menu-item {
    justify-content: center;
    padding: 0 !important;
  }

  .side-menu .el-menu-item .el-icon {
    margin: 0;
  }
}
</style>
