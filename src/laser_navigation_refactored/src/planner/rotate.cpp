#include  "laser_navigation_refactored/planner/rotate.hpp"

#include <cmath>
#include <iostream>

namespace rotate
{
    void TranslateRotate::setTranslateTask(const TranslateTask& task)
    {
        translate_task = task;
        if(task.linear < 0)
        {
            translate_task.acc *= -1;
            translate_task.dec *= -1;
        }

        task_status = TaskStatus::tranlating;
        flag_init = true;
        flag_dec = false;

        //time1 time2 time3分别为匀加速时间，匀速直线运动时间，匀减速时间
        time1 = translate_task.linear / translate_task.acc;
        //double time2; 
        time3 = translate_task.linear / translate_task.dec;

        s1 = fabs((1/2.0)*translate_task.acc*pow(time1,2));
        s3 = fabs((1/2.0)*translate_task.dec*pow(time3,2));
        s2 = fabs(translate_task.distance - (s1 + s3));

        //参数错误
        if(s1 + s3 > translate_task.distance)
        {
            s1 = s3 = translate_task.distance / 2.0;
            s2 = 0;
            time1 = sqrt(2*s1/translate_task.acc);
            translate_task.linear = time1 * translate_task.acc;
        }
    }

    void TranslateRotate::setRotateTask(const RotateTask& task)
    {
        rotate_task = task;
        task_status = TaskStatus::rotating;
        flag_init = true;
        flag_dec = false;
        angle_counter = 0.0;

        if(task.angular < 0)
        {
            rotate_task.acc *= -1;
            rotate_task.dec *= -1;
        }

        time1 = rotate_task.angular / rotate_task.acc;
        //double time2; 
        time3 = rotate_task.angular / rotate_task.dec;

        s1 = fabs((1/2.0) * rotate_task.acc * pow(time1,2));
        s3 = fabs((1/2.0) * rotate_task.dec * pow(time3,2));
        s2 = fabs(rotate_task.theta - (s1 + s3));

        //参数错误
        if(s1 + s3 > rotate_task.theta)
        {
            s1 = s3 = rotate_task.theta / 2.0;
            s2 = 0;
            time1 = sqrt(2*s1/rotate_task.acc);
            rotate_task.angular = time1 * rotate_task.acc;
        }
    }

    double TranslateRotate::getDistance(Pose2dtr pose)
    {
        double x = fabs(pose.x - init_pose.x);
        double y = fabs(pose.y - init_pose.y);
        return sqrt(x*x + y*y); 
    }

    Result TranslateRotate::getSpeed(const Pose2dtr& pose)
    {
        Result speed;
        speed.status = task_status;
        
        if(flag_init)
        {
            init_pose = pose;
            flag_init = false;
            speed.status = task_status;
            last_speed = speed;
            last_angle = pose.yaw;
            return speed;
        }

        if(task_status == TaskStatus::tranlating)
        {
            if(fabs(translate_task.acc) < 1e-6 || fabs(translate_task.dec) < 1e-6)
            {
                task_status = TaskStatus::error;
                speed.status = task_status;
                std::cout << "error" << std::endl;
                return speed;
            }
            //匀加速直线
            if(getDistance(pose) < s1)
            {   
                std::cout << "add" << std::endl;
                speed.linear = last_speed.linear + translate_task.acc / (1000 /translate_task.timer_interval);
                speed.angular = 0.0;
                if(fabs(speed.linear) > fabs(translate_task.linear))
                {
                    speed.linear = translate_task.linear;
                    std::cout << "add hold" << std::endl;
                }
            }
            //匀速直线
            else if(s1 < getDistance(pose) && getDistance(pose) < s1 + s2 - fabs(translate_task.linear) / (1000 / translate_task.timer_interval))
            {
                std::cout << "--%" << std::endl;
                speed.linear = translate_task.linear;
                speed.angular = 0.0;
            }
            //匀减速直线
            else if((getDistance(pose) >s1 || getDistance(pose) > s1 + s2 - fabs(translate_task.linear) / (1000 / translate_task.timer_interval)) 
                      && fabs(getDistance(pose)- translate_task.distance) > translate_task.reach_dist)
            {
                std::cout << "dec" << std::endl;
                speed.linear = last_speed.linear - translate_task.dec / (1000/translate_task.timer_interval);
                speed.angular = 0.0;
                //防止机器人倒退
                if(speed.linear * last_speed.linear < 0 || fabs(speed.linear) < translate_task.reach_dist)
                {
                    speed.linear = last_speed.linear;
                    std::cout << "dec hold" << std::endl;
                }
            }

            //结束条件：
            // std::cout << "debug:" << std::endl;
            // std::cout << getDistance(pose) << std::endl;
            // std::cout << translate_task.distance << std::endl;
            // std::cout << (fabs(getDistance(pose) - translate_task.distance) < translate_task.reach_dist) << std::endl;
            // std::cout << (getDistance(pose) > translate_task.distance) << std::endl;
            // std::cout << "------" << std::endl;
            // std::cout << (getDistance(pose) > s1 + s2 - fabs(translate_task.linear) / (1000 / translate_task.timer_interval)) << std::endl;  
            // std::cout << (fabs(getDistance(pose)- translate_task.distance) > translate_task.reach_dist) << std::endl;
            // std::cout << "------" << std::endl;

            if(fabs(getDistance(pose) - translate_task.distance) < translate_task.reach_dist || getDistance(pose) > translate_task.distance)
            {
                speed.linear = 0.0;
                speed.angular = 0.0;
                task_status = TaskStatus::finish;
                speed.status = task_status;
                std::cout << "finished" << std::endl;
            }
        }   
        else if(task_status == TaskStatus::rotating)
        {
            if(fabs(rotate_task.acc) < 1e-6 || fabs(rotate_task.dec) < 1e-6)
            {
                speed.linear = 0.0;
                speed.angular = 0.0;
                task_status = TaskStatus::error;
                speed.status = task_status;
                last_speed = speed;
                return speed;
            }
 
            double angle = pose.yaw - last_angle;
            if(angle > M_PI)
            {
                angle -= 2 * M_PI;
            }
            else if(angle < -M_PI)
            {
                angle += 2 * M_PI;
            }
            angle_counter += angle;

            //匀加速转动
            if(fabs(angle_counter) < s1)
            {
                std::cout << "add" << std::endl;
                speed.linear = 0.0;
                speed.angular = last_speed.angular + rotate_task.acc / (1000/rotate_task.timer_interval);
            }
            //匀速转动
            else if(fabs(angle_counter) < s1 + s2 - fabs(rotate_task.angular) / (1000/rotate_task.timer_interval))
            {
                std::cout << "%%" << std::endl;
                speed.linear = 0.0;
                speed.angular = rotate_task.angular;
            }
            //匀减速转动
            else if((fabs(angle_counter) > s1 || fabs(angle_counter) >  s1 + s2 - fabs(rotate_task.angular) / (1000/rotate_task.timer_interval))
                    && fabs(fabs(angle_counter) - rotate_task.theta) > rotate_task.reach_angle)
            {
                std::cout << "dec" << std::endl;
                speed.linear = 0.0;
                speed.angular = last_speed.angular - rotate_task.dec / (1000/rotate_task.timer_interval);

                //防止机器人往回转
                if(last_speed.angular * speed.angular < 0)
                {
                    speed.angular = last_speed.angular;
                }
            }
        
            //结束条件
            std::cout << "debug:" << std::endl;
            std::cout << angle_counter << std::endl;
            std::cout << rotate_task.theta << std::endl;
            std::cout << (fabs(angle_counter - rotate_task.theta) < rotate_task.reach_angle) << std::endl;
            std::cout << (fabs(angle_counter) > rotate_task.theta) << std::endl;
            std::cout << "------" << std::endl;
            std::cout << (fabs(angle_counter) >  s1 + s2 - fabs(rotate_task.angular) / (1000/rotate_task.timer_interval)) << std::endl;
            std::cout << (fabs(fabs(angle_counter) - rotate_task.theta) > rotate_task.reach_angle) << std::endl;
            std::cout << "------" << std::endl;
            if(fabs(fabs(angle_counter) - rotate_task.theta) < rotate_task.reach_angle || fabs(angle_counter) > rotate_task.theta)
            {
                task_status = TaskStatus::finish;
                speed.linear = 0.0;
                speed.angular = 0.0;
                speed.status = task_status;
            }
        }

        last_speed = speed;
        last_angle = pose.yaw;
        return speed;
    }

    Result TranslateRotate::getSpeed(double line_dec,double angle_dec)
    {
        Result speed;
        double line_speed = 0.0, angle_speed = 0.0;
        double time_invental = 50;
        if(last_speed.angular > 1e-5)
        {
            angle_speed = last_speed.angular - fabs(angle_dec) / time_invental;
            if(angle_speed * last_speed.angular < 0 || fabs(angle_speed) < 1e-5)
            {
                angle_speed = 0.0;
            }
        }
        else if(last_speed.angular < 1e-5)
        {
            angle_speed = last_speed.angular + fabs(angle_dec) / time_invental;
            if(angle_speed * last_speed.angular < 0 || fabs(angle_speed) < 1e-5)
            {
                angle_speed = 0.0;
            }
        }

        if(last_speed.linear > 1e-5)
        {
            line_speed = last_speed.linear - fabs(line_dec) / time_invental;
            if(line_speed * last_speed.linear < 0 || fabs(line_speed) < 1e-5)
            {
                line_speed = 0.0;
            }
        }
        else if(last_speed.linear < 1e-5)
        {
            line_speed = last_speed.linear + fabs(line_dec) / time_invental;
            if(line_speed * last_speed.linear < 0 || fabs(line_speed) < 1e-5)
            {
                line_speed = 0.0;
            }
        }

        if(fabs(angle_speed) < 1e-5 && fabs(line_speed) < 1e-5)
        {
            task_status = TaskStatus::finish;
        }
        else
        {
            task_status = last_speed.status;
        }

        speed.angular = angle_speed;
        speed.linear = line_speed;
        speed.status = task_status;
        std::cout << "curr_speed： " << angle_speed << " " << line_speed << std::endl;
        last_speed = speed;
        return speed;
    }
}
