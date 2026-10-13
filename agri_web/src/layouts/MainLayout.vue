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
  min-width: 1024px;
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
  margin-left: 228px;
  width: calc(100% - 228px);
}

.main-content {
  padding: 24px;
  overflow-x: hidden;
  overflow-y: auto;
  background: #f4f7fb;
}
</style>
