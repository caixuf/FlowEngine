#!/bin/bash

# 测试任务系统的脚本
echo "=== 测试任务系统 ==="

cd build

# 创建测试输入
echo "start task1
status task1  
start task2
list
stats
health
quit" | ./flow_task
