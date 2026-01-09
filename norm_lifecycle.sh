#!/bin/bash

NAMESPACE=$1
if [ -n "$NAMESPACE" ]; then
  NAMESPACE="/$NAMESPACE"
fi

# 状態遷移可能かチェック（リトライ機能付き）
transition() {
	local node=$1
	local transition=$2
    local max_retries=3
    local retry=0
    
    echo "Transitioning node: $node to $transition"
    
    # ノードが存在するまで待機
    while [ $retry -lt $max_retries ]; do
        local state=$(timeout 5 ros2 lifecycle get $node 2>/dev/null | grep -oE '^[a-z_]+')
        if [[ -n "$state" ]]; then
            break
        fi
        retry=$((retry + 1))
        echo "  Retry $retry/$max_retries: Waiting for node to be available..."
        sleep 1
    done
    
    if [[ -z "$state" ]]; then
        echo "  Warning: Could not get state for $node (timeout or not responding)"
        return 1
    fi
    
    echo "  Current state: $state"
	if [[ "$transition" == "configure" ]] && [[ "$state" == "unconfigured" ]]; then
        echo "  Executing: configure"
        if timeout 20 ros2 lifecycle set $node configure 2>&1; then
            echo "  Transitioning successful"
            return 0
        else
            echo "  Error: configure failed or timeout"
            return 1
        fi
	elif [[ "$transition" == "activate" ]] && [[ "$state" == "inactive" ]]; then
        echo "  Executing: activate"
        if timeout 20 ros2 lifecycle set $node activate 2>&1; then
            echo "  Transitioning successful"
            return 0
        else
            echo "  Error: activate failed or timeout"
            return 1
        fi
	else
        echo "  Skipping: already in correct state or invalid transition"
        return 0
	fi
}


# 各ノードの遷移（順序重要）
# 1. コストマップをconfigure
transition ${NAMESPACE}/global_costmap/global_costmap configure
transition ${NAMESPACE}/local_costmap/local_costmap configure

# 2. ナビゲーションコンポーネントをconfigure
transition ${NAMESPACE}/bt_navigator configure
transition ${NAMESPACE}/behavior_server configure
transition ${NAMESPACE}/planner_server configure
transition ${NAMESPACE}/controller_server configure

# 3. bt_navigatorをactivate
transition ${NAMESPACE}/bt_navigator activate

# 4. controller_serverとlocal_costmapをactivate（順序重要）
transition ${NAMESPACE}/controller_server activate
sleep 1  # controller_serverが完全に起動するまで待機

# 5. behavior_serverをactivate
transition ${NAMESPACE}/behavior_server activate

# 6. global_costmapをactivate
transition ${NAMESPACE}/global_costmap/global_costmap activate

# 7. local_costmapをactivate
transition ${NAMESPACE}/local_costmap/local_costmap activate
sleep 1  # costmapが初期化されるまで待機

# 8. planner_serverをactivate（これが最後に必要）
transition ${NAMESPACE}/planner_server activate
sleep 1  # planner_serverが完全に起動するまで待機

# # route_serverは設定エラーが発生する可能性があるが、必須ではないのでエラーを無視
# transition ${NAMESPACE}/route_server configure || echo "  Note: route_server configure failed (may not be critical)"
# transition ${NAMESPACE}/route_server activate || echo "  Note: route_server activate failed (may not be critical)"

transition ${NAMESPACE}/velocity_smoother configure
transition ${NAMESPACE}/collision_monitor configure
transition ${NAMESPACE}/velocity_smoother activate
transition ${NAMESPACE}/collision_monitor activate