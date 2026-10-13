<template>
  <div class="welcome-banner">
    <div class="welcome-label">
      AGRICULTURAL ROBOTICS SYSTEM
    </div>

    <h2>让农业作业更加智能、高效</h2>

    <p>
      统一监控机器人状态、管理作业任务与查看农田作业进度。
    </p>

    <el-tag effect="dark" class="demo-tag">
      前端演示版本
    </el-tag>
  </div>
</template>

<style scoped>
.welcome-banner {
  padding: 27px 34px;
  margin-bottom: 28px;
  border-radius: 12px;
  background: linear-gradient(
    112deg,
    #1b674b,
    #25845c 60%,
    #42a276
  );
  color: white;
}

.welcome-label {
  color: #c5ead8;
  font-size: 10px;
  letter-spacing: 1.8px;
}

.welcome-banner h2 {
  margin: 12px 0 9px;
  font-size: 23px;
}

.welcome-banner p {
  margin-bottom: 15px;
  color: #d7eee2;
  font-size: 12px;
}

.demo-tag {
  border-color: rgb(255 255 255 / 25%) !important;
  background: rgb(255 255 255 / 12%) !important;
  color: white !important;
}

@media (max-width: 760px) {
  .welcome-banner {
    padding: 22px;
  }

  .welcome-banner h2 {
    font-size: 19px;
  }
}
</style>
