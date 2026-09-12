#!/bin/bash

echo "===== 当前状态 ====="
git status

echo ""
echo "===== 获取远程更新 ====="
git pull origin main

echo ""
echo "===== 更新完成 ====="