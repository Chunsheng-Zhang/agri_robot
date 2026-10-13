import { createRouter, createWebHistory } from 'vue-router'
import MainLayout from '../layouts/MainLayout.vue'

const router = createRouter({
  history: createWebHistory(import.meta.env.BASE_URL),

  routes: [
    {
      path: '/',
      component: MainLayout,
      children: [
        {
          path: '',
          name: 'overview',
          component: () => import('../views/DashboardView.vue'),
          meta: { title: '系统总览' },
        },
        {
          path: 'monitor',
          name: 'monitor',
          component: () => import('../views/PlaceholderView.vue'),
          meta: { title: '作业监控' },
        },
        {
          path: 'tasks',
          name: 'tasks',
          component: () => import('../views/PlaceholderView.vue'),
          meta: { title: '任务管理' },
        },
        {
          path: 'planning',
          name: 'planning',
          component: () => import('../views/PlaceholderView.vue'),
          meta: { title: '路径规划' },
        },
        {
          path: 'devices',
          name: 'devices',
          component: () => import('../views/PlaceholderView.vue'),
          meta: { title: '设备管理' },
        },
        {
          path: 'collaboration',
          name: 'collaboration',
          component: () => import('../views/PlaceholderView.vue'),
          meta: { title: '协同作业' },
        },
        {
          path: 'settings',
          name: 'settings',
          component: () => import('../views/PlaceholderView.vue'),
          meta: { title: '系统设置' },
        },
      ],
    },
  ],

  scrollBehavior() {
    return { top: 0 }
  },
})

export default router
