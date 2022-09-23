#include "dwa.h"


DWA::DWA()
{

}
DWA::DWA(Environment* env, QPointF start, QPointF destination)
{
    startPoint = start;
    destinationPoint = destination;
    environment = env;
    destinationState.x = destination.x();
    destinationState.y = destination.y();
    destinationState.x = 30;
    destinationState.y = 50;
    destinationState.yaw = 0;
    destinationState.speed = 0;
    destinationState.angular_speed = 0;
}
//路径规划
void DWA::planning()
{
    CarState currentState(startPoint.x(), startPoint.y(), 0, 0, 0);
    vector<CarState> currentTrajectory;
    while(1)
    {
        cout << "**********************************************" << endl;
        vector<float> speed(2);     //v[0]为速度, v[1]角速度
        speed = dwa_control(currentState);      
        cout << "speed:" << speed[0] << ", " << speed[1] << endl;
        currentTrajectory.clear();
        aaa = false;
        predict_trajectory(currentState, speed[0], speed[1], currentTrajectory);
        aaa = false;
        trajectory.push_back(currentTrajectory);
        currentState = currentTrajectory.back();
        //判断是否到达终点
        if(pow(currentState.x - destinationState.x, 2) + pow(currentState.y - destinationState.y, 2) <= car.radius * car.radius)
        {
            cout << "Done" << endl;
            break;
        }
        cout << "currentState:(" << currentState.x << ", " << currentState.y << ", " << currentState.yaw * 180 / PI << ")" << currentState.speed << "  " << currentState.angular_speed << endl;
    }
}
//动态窗口法
vector<float> DWA::dwa_control(const CarState &carstate)
{
    vector<float> dw(4);     //dw[0]为最小速度，dw[1]为最大速度，dw[2]为最小角速度，dw[3]为最大角速度
    //计算动态窗口
    dw = calc_dw(carstate);
    //计算最佳（v, w）
    vector<float> v_w(2);
    v_w = calc_best_speed(carstate, dw);
    return v_w;
}
// 计算动态窗口
vector<float> DWA::calc_dw(const CarState &carstate)
{
    // 机器人速度属性限制的动态窗口
    vector<float> dw_robot_state{car.min_speed, car.max_speed, car.min_angular_speed, car.max_angular_speed};
    // 机器人模型限制的动态窗口
    vector<float> dw_robot_model(4);
    dw_robot_model[0] = carstate.speed - car.max_accel * car.dt;
    dw_robot_model[1] = carstate.speed + car.max_accel * car.dt;
    dw_robot_model[2] = carstate.angular_speed - car.max_angular_speed_rate * car.dt;
    dw_robot_model[3] = carstate.angular_speed + car.max_angular_speed_rate * car.dt;
    vector<float> dw{max(dw_robot_state[0], dw_robot_model[0]),
                     min(dw_robot_state[1], dw_robot_model[1]),
                     max(dw_robot_state[2], dw_robot_model[2]),
                     min(dw_robot_state[3], dw_robot_model[3])};
    return dw;
}
//在dw中计算最佳速度和角速度
vector<float> DWA::calc_best_speed(const CarState &carstate, const vector<float> &dw)
{
    vector<float> best_speed{0, 0};
    vector<CarState> trajectoryTmp;
    float min_cost = 10000;
    float final_cost;
    float goal_cost;
    float speed_cost = 0;
    float obstacle_cost = 0;
    float distance_cost = 0;
    for(float i = dw[0]; i < dw[1]; i += car.v_resolution)
    {
        for (float j = dw[2]; j < dw[3]; j += car.yaw_rate_resolution)
        {
            //预测轨迹
            trajectoryTmp.clear();
            predict_trajectory(carstate, i, j, trajectoryTmp);
            //计算代价
            goal_cost = car.goal_cost_gain * calc_goal_cost(trajectoryTmp);
            speed_cost = car.speed_cost_gain * (car.max_speed - trajectoryTmp.back().speed);
            obstacle_cost = car.obstacle_cost_gain * calc_obstacle_cost(trajectoryTmp);
            distance_cost = 0.1 * sqrt(pow(destinationState.x - trajectoryTmp.back().x, 2) + pow(destinationState.y - trajectoryTmp.back().y, 2));
            final_cost = goal_cost + speed_cost + obstacle_cost + distance_cost;

            if(final_cost < min_cost)
            {
                min_cost = final_cost;
                best_speed[0] = i;
                best_speed[1] = j;
            }
            if(best_speed[0] < 0.001 && carstate.speed < 0.001)
                best_speed[1] = -car.max_angular_speed_rate;
        }
    }
    //cout << "best_speed:" << best_speed[0] << ",   " << best_speed[1] << endl;
    return best_speed;
}
// 在一段时间内预测轨迹
void DWA::predict_trajectory(const CarState &carstate, const float &speed, const float &angular_speed, vector<CarState> &trajectory)
{
    float time = 0;
    CarState nextState = carstate;
    nextState.speed = speed;
    nextState.angular_speed = angular_speed;
    while(time < car.predict_time)
    {
        nextState = motion_model(nextState, speed, angular_speed);
        if(aaa)
            cout << "nextState:(" << nextState.x << ", " << nextState.y << ", " << nextState.yaw * 180 / PI << ")" << nextState.speed << "  " << nextState.angular_speed << endl;
        trajectory.push_back(nextState);
        time += car.dt;
    }
}
//根据动力学模型计算下一时刻状态
CarState DWA::motion_model(const CarState &carstate, const float &speed, const float &angular_speed)
{
    CarState nextState;
    nextState.x = carstate.x + speed * car.dt * cos(carstate.yaw);
    nextState.y = carstate.y + speed * car.dt * sin(carstate.yaw);
    nextState.yaw = carstate.yaw + angular_speed * car.dt;
    nextState.speed = carstate.speed;
    nextState.angular_speed = carstate.angular_speed;
    return nextState;
}
// 计算方位角代价
float DWA::calc_goal_cost(const vector<CarState> &trajectory)
{
    float error_yaw = atan2(destinationState.y - trajectory.back().y, destinationState.x - trajectory.back().x);
    float goal_cost = error_yaw - trajectory.back().yaw;
//    cout << "error_yaw :" << error_yaw << "    yaw:" << trajectory.back().yaw;
    goal_cost = atan2(sin(goal_cost), cos(goal_cost));
//    cout << "    final:" << goal_cost << endl;
    if(goal_cost >= 0)
        return goal_cost;
    else
        return -goal_cost;
}
// 计算障碍代价
float DWA::calc_obstacle_cost(const vector<CarState> &trajectory)
{
    //float obstacle_cost;
    float distance;
    for (int i = 0; i < environment->barrier.size(); i ++) {
        for (int j = 0; j < trajectory.size(); j ++) {
            distance = sqrt(pow(environment->barrier[i].x() - trajectory[j].x, 2) + pow(environment->barrier[i].y() - trajectory[j].y, 2));
            if(distance <= car.radius)
                return 10000.0;
        }
    }
    return 0;
}
