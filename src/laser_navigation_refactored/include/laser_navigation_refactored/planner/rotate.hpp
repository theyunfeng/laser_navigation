#ifndef __ROTATE_H__
#define __ROTATE_H__

namespace rotate
{

    struct TranslateTask
    {
        double acc = 0.0;
        double dec = 0.0;
        double linear = 0.0;
        double distance = 0.0;       // 单位：m
        double timer_interval = 20;  // 单位：ms
        double reach_dist = 0.005;
    };

    struct RotateTask
    {
        double acc = 0.0;
        double dec = 0.0;
        double angular = 0.0;
        double theta = 0.0;           // 单位：角度
        double timer_interval = 20;   // 单位：ms
        double reach_angle = 0.005;
    };

    enum TaskStatus
    {
        finish,     //无任务
        tranlating, //平动中
        rotating,   //旋转
        error       //出错
    };

    struct Result
    {
        double linear = 0.0;
        double angular = 0.0;
        TaskStatus status = TaskStatus::finish;
    };

    struct Pose2dtr
    {
        double x;
        double y;
        double yaw;
    };

    class TranslateRotate
    {
    public:
        void setTranslateTask(const TranslateTask& task);
        void setRotateTask(const RotateTask& task);

        Result getSpeed(const Pose2dtr& pose);
        Result getSpeed(double line_dec,double angle_dec);

    protected:
        double getDistance(Pose2dtr pose);

    private:
        TaskStatus task_status = TaskStatus::finish;
        TranslateTask translate_task;
        RotateTask rotate_task;
        bool flag_init = true;
        Pose2dtr init_pose;
        Result last_speed;
        bool flag_dec = false;
        //以累加的形式进行旋转
        double last_angle;
        double angle_counter = 0.0;

        double time1,time3;
        double s1,s2,s3;
    };

} // namespace rotate

#endif // __ROTATE_H__
