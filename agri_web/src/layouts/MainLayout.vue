<template>
  <el-container class="app-layout">
    <AppSidebar />

    <el-container class="main-container" direction="vertical">
      <AppHeader />

      <el-main class="main-content">
        <router-view />
      </el-main>
    </el-container>
  </el-container>
</template>

<script setup lang="ts">
import AppSidebar from '../components/AppSidebar.vue'
import AppHeader from '../components/AppHeader.vue'
</script>

<style>
* {
  box-sizing: border-box;
}

html,
body,
#app {
  width: 100%;
  min-width: 320px;
  min-height: 100%;
  margin: 0;
}

body {
  font-family:
    -apple-system, BlinkMacSystemFont, "Segoe UI",
    "Microsoft YaHei", sans-serif;
  color: #263445;
  background: #f4f7fb;
}

.app-layout {
  min-height: 100vh;
}

.main-container {
  min-width: 0;
  min-height: 100vh;
  height: 100vh;
  margin-left: 228px;
  width: calc(100% - 228px);
}

.main-content {
  min-height: 0;
  padding: 24px;
  overflow-x: hidden;
  overflow-y: auto;
  background: #f4f7fb;
}

@media (max-width: 800px) {
  .sidebar { width: 64px !important; }
  .sidebar .brand { padding: 18px 8px; }
  .sidebar .brand > div:not(.brand-logo), .sidebar .menu-caption, .sidebar .sidebar-bottom,
  .sidebar .side-menu .el-menu-item > span { display: none; }
  .sidebar .side-menu .el-menu-item { padding-left: 18px !important; }
  .main-container { margin-left: 64px; width: calc(100% - 64px); }
  .main-content { padding: 12px; }
  .topbar { height: auto !important; min-height: 66px; padding: 10px 12px !important; flex-wrap: wrap; gap: 8px; }
  .topbar .breadcrumb { display: none; }
}
</style>
