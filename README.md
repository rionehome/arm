# arm
## Overview

Crane+v2 用のパッケージ

## Setup
詳細はここに書いてある

https://gbiggs.github.io/ros_moveit_rsj_tutorial/manipulators_and_moveit.html

moveitをダウンロード
```
sudo apt-get install ros-kinetic-moveit-*
```
crane_plusのパッケージをGithubからclone
```
git clone https://github.com/gbiggs/crane_plus_arm.git
```
その後 catkin_makeを忘れずに行う
```
catkin_make
```


## Usage

```
roslaunch arm arm.launch
```

/arm/controlにstringで何でもいいので投げると、

基本の形に移動 -> (2秒待機) -> 手を広げる -> (2秒待機) -> 手を閉じる

のような挙動をする.

## Node
**`name` sound_system**
