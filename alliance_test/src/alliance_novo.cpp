#include <alliance_msgs/Message.h>
#include <alliance_msgs/Motivation.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sstream>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#define PI 3.14159265

// Posição inicial do robô na simulação
double xi = -5.0;
double yi = -5.0;
double thi = 0.0;

int id = 0;
double vel_x;
double vel_omega;
double k = 0.8;
double alpha, beta;
// int loop = 0;
geometry_msgs::Twist vel;
nav_msgs::Odometry odom_msg;

float lastx = 0.0;
float lasty = 0.0;
float d = 0.0;
float d_wand = 0.0;
bool enable_d = true;
bool enable_e = true;
bool see_wall = false;
int pos_count = 0;
int wander_count = 0;
int pos_wall_initx, pos_wall_inity;
double current_posx, current_posy;
double roll, pitch, yaw;
double sonar0, sonar1, sonar2, sonar3, sonar4, sonar5, sonar6, sonar7;
// double sonar0x, sonar1x, sonar2x, sonar3x, sonar4x, sonar5x, sonar6x,
// sonar7x;
// double sonar0y, sonar1y, sonar2y, sonar3y, sonar4y, sonar5y, sonar6y,
// sonar7y;

// Distâncias do robô medidas pelo sonar
double vcloser = 0.5;
double closer = 1.0;
double inter = 1.6;
double far = 2.5;
double vfar = 3.0;

// Definição das variáveis envolvidas na arquitetura alliance
// Threshold de ativação de um comportamento
int theta = 200;
// Variável de entrada que avalia a partir dos sensores se o comportamento é
// aplicável (true) naquele instante de tempo
int sensory_feedback[4] = {1, 1, 1, 1};
// Variável que define a frequência de envio de mensagens por unidade de tempo
// do robô1, de acordo com sua atividade
float rho1 = 10;
// Variável que define quanto tempo o robô1 pode passar sem receber mensagens de
// outros robôs, antes de assumir que o determinado robô cessou sua atividade
int tau1 = 20;
// Variável que define se determinado comportamento deve ser suprimido dado que
// outro comportamento está ativo
int activity_suppression[4];
// Variável de comunicação recebida de cada robô sobre cada
// tarefa(comportamento)
int comm[4][3], comm_phi[4][3];

// Variáveis de impaciência do robô
// Variável de tempo que determina quanto tempo o robô1 está disposto a permitir
// que as mensagens enviadas pelos outros robôs afetem a
// motivação do comportamento
double phi = 1000;
// Variável que determina uma variação lenta na impaciência, após o robô1
// identificar que outro robô está realizando uma tarefa
double delta_slow[4] = {1.0, 2.5, 5.0, 2.0};
// Variável que determina uma variação rápida na impaciência, quando não há robô
// realizando determinada tarefa
double delta_fast[4] = {1.0, 5.0, 10.0, 3.0};
// Variável de impaciência
double impatience[4] = {1.0, 5.0, 10.0, 3.0};
// Variável de reset de impaciência
int impatience_reset[4] = {1, 1, 1, 1};
// int task_counter[4][3] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int task_done[4] = {0, 0, 0, 0};
// Variáveis de aquiescência
// Variável que determina o tempo que o robô1 manterá um comportamento antes de
// passá-la a outro robô
int psi[4] = {10, 1000, 500, 120}; // 2 minutos
// Variável de aquiescência
int acquiescence[4] = {1, 1, 1, 1};
// Variável que determina o tempo que o robô1 manterá um comportamento antes
// tentar realizar outro
double lambda[4] = {720, 1000, 420, 420};

// Motivação
double m1[4] = {0.0, 0.0, 0.0, 0.0};
// Motivação anterior
float m1_old[4] = {0.0, 0.0, 0.0, 0.0};

ros::Time current_time, last_time;
int current_task, my_task;
int current_robot;
int current_behaviour = 0;
ros::Time init_stamp;
ros::Time timenow;
ros::Time old_stamp[4][3];
ros::Duration diff_stamp[4][3];
int active_time[4] = {0, 0, 0, 0};
int j = 0;
int cur_x = 0.0;
int cur_y = 0.0;
double d_init = 0.0;
int task_done1 = 0;
int task_done2 = 0;
int task_done3 = 0;
double goal_d = 0.0;
double goal_d_old = 0.0;
float th = 0.0;
int report_progress = 0;
int odom_feedback = 0;
int range_feedback = 0;

geometry_msgs::Pose current_pos;

std_msgs::Header header;
alliance_msgs::Message taskmsg;
alliance_msgs::Motivation motiv;
geometry_msgs::PoseStamped current_position;

void task_filler()
{
  taskmsg.header.stamp = ros::Time::now();
  taskmsg.task_id = current_behaviour;
  taskmsg.robot_id = id;
  taskmsg.task_done1 = task_done[1];
  taskmsg.task_done2 = task_done[2];
  taskmsg.task_done3 = task_done[3];
}

void report()
{
  ROS_INFO("REPORT: Tasks WANDER and BORDER PROTECTION are being done.");
  double x_goal(11.0);
  double y_goal(7.0);
  goal_d = sqrt((x_goal - current_position.pose.position.x) *
                    (x_goal - current_position.pose.position.x) +
                (y_goal - current_position.pose.position.y) *
                    (y_goal - current_position.pose.position.y));
  beta = atan2((y_goal - current_position.pose.position.y) *
                   (y_goal - current_position.pose.position.y),
               (x_goal - current_position.pose.position.x) *
                   (x_goal - current_position.pose.position.x));
  alpha = -th + beta;
  vel_x = 0.3 * goal_d;
  vel_omega = 8.0 * alpha - 1.0 * beta;
  ROS_INFO("Goal_d: %f, Alpha: %f, Beta: %f", goal_d, alpha, beta);
  if (sonar2 < closer || sonar3 < closer || sonar0 < vcloser ||
      sonar5 < vcloser)
  {
    vel_x = 2.5 * ((std::max(sonar2, sonar3)) - (1 / std::max(sonar2, sonar3)));
    vel_omega = 5.0 * k * (std::min((sonar0), std::min((sonar1), (sonar2))) -
                           (std::min((sonar3), std::min((sonar4), (sonar5)))));
  }
  else if ((std::min(sonar0, std::min(sonar1, sonar2)) -
            std::min(sonar3, std::min(sonar4, sonar5))) < 0.02 &&
           std::min(sonar0, std::min(sonar1, sonar2)) -
                   std::min(sonar3, std::min(sonar4, sonar5)) >
               -0.02)
  {
    vel_x = -10;
    vel_omega = 10;
  }
  if (vel_x < 0.05 && vel_x > -0.05)
  {
    vel_x = 0.0000;
    vel_omega = 0.0000;
  }
  if (goal_d < 0.3)
  {
    report_progress++;
  }
  vel.linear.x = vel_x;
  vel.angular.z = vel_omega;
  task_filler();
}

void boundary_overwatch()
{
  ROS_INFO("REPORT: Task BORDER PROTECTION is being done.");
  // Achando a parede: WANDER enquanto a parede está longe
  if (current_position.pose.position.x < 9.0 &&
      current_position.pose.position.x > -7.0 &&
      current_position.pose.position.y < 6.0 &&
      current_position.pose.position.y > -6.0)
  {
    vel_x = k * (std::max(sonar2, sonar3) - 1 / std::max(sonar2, sonar3));
    vel_omega = 0.5 * k / std::min(sonar2, sonar3) *
                (std::min(sonar0, std::min(sonar1, sonar2)) -
                 std::min(sonar3, std::min(sonar4, sonar5)));
    ROS_INFO("Vel_omega %0.2f.", vel_omega);
    vel.linear.x = vel_x;
    vel.angular.z = vel_omega;
  }
  // SEGUINDO A PAREDE
  // Pela esquerda
  else if (sonar0 < sonar5 && enable_e)
  {
    enable_d = false;
    enable_e = true;
    vel_x = 0.5; // anterior 0.3
    vel_omega = 0.4 * (sonar0 - vcloser) + 1.2 * (sonar1 - 0.75 * sonar7) +
                0.3 * (sonar3 - std::max(sonar4, std::max(sonar6, sonar5)));
  }
  // Pela direita
  else if (sonar5 < sonar0 && enable_d)
  {
    enable_e = false;
    enable_d = true;
    vel_x = 0.5; // anterior 0.3
    vel_omega = -0.4 * (sonar5 - vcloser) - 1.2 * (sonar4 - 0.75 * sonar6) -
                0.3 * (sonar2 - std::max(sonar1, std::max(sonar7, sonar0)));
  }
  vel.linear.x = vel_x;
  vel.angular.z = vel_omega;
}

// WANDER: robô vaga desviando de obstáculos
void wander()
{
  ROS_INFO("REPORT: Task WANDER is being done.");
  if (sonar0 > inter && sonar1 > inter && sonar2 > inter && sonar3 > inter &&
      sonar4 > inter && sonar5 > inter)
  {
    vel_x = 5 * k * (std::max(sonar2, sonar3) - 1 / std::max(sonar2, sonar3));
    vel_omega = 0.005 * k / std::max(sonar2, sonar3) *
                (std::min(sonar0, std::min(sonar1, sonar2)) -
                 std::min(sonar3, std::min(sonar4, sonar5)));
  }
  else if ((std::min(sonar0, std::min(sonar1, sonar2)) -
            std::min(sonar3, std::min(sonar4, sonar5))) < 0.02 &&
           (std::min(sonar0, std::min(sonar1, sonar2)) -
            std::min(sonar3, std::min(sonar4, sonar5))) > -0.02)
  {
    vel_x = -10;
    vel_omega = 10;
  }
  else
  {
    vel_x = 2 * k * (std::max(sonar2, sonar3) - 1 / std::max(sonar2, sonar3));
    vel_omega = 0.5 * k / std::min(sonar2, sonar3) *
                (std::min(sonar0, std::min(sonar1, sonar2)) -
                 std::min(sonar3, std::min(sonar4, sonar5)));
  }
  ROS_INFO("Vel_omega %0.2f.", vel_omega);
  vel.linear.x = vel_x;
  vel.angular.z = vel_omega;
}

void suppression()
{
  if (current_behaviour == 0 || acquiescence[my_task] == 0)
  {
    ROS_INFO("Suppression in");
    activity_suppression[1] = 1 - task_done[1];
    activity_suppression[2] = 1 - task_done[2];
    activity_suppression[3] = 1 - task_done[3];
    impatience_reset[1] = 1;
    impatience_reset[2] = 1;
    impatience_reset[3] = 1;
  }
  else if (my_task != 0)
  {
    ROS_INFO("Suppression in2");
    for (int i = 1; i <= 3; i++)
    {
      activity_suppression[i] = my_task == i && task_done[i] == 0 ? 1 : 0;
    }
  }
}

void idle()
{
  ROS_INFO("REPORT: Task IDLE is being done.");
  vel_x = 0;
  vel_omega = 0;
  vel.linear.x = vel_x;
  vel.angular.z = vel_omega;
  suppression();
}

void behaviour_set()
{
  // ROS_INFO("Behaviour_set");
  for (int i = 1; i <= 3; i++)
  {
    if (m1[i] >= theta)
    {
      current_behaviour = i;
      m1[i] = theta;
    }
    else
    {
      active_time[i] = 0;
    }
  }
  if (m1[1] < theta && m1[2] < theta && m1[3] < theta)
  {
    current_behaviour = 0;
  }
  suppression();
}

void motivation_calc()
{
  // ROS_INFO("Motivation_calc");
  for (int i = 1; i <= 3; i++)
  {
    m1[i] = (m1[i] + impatience[i]) * sensory_feedback[i] *
            activity_suppression[i] * impatience_reset[i] * acquiescence[i];
    ROS_INFO("impatience %d %0.2f", i, impatience[i]);
    ROS_INFO("sensory%d %d", i, sensory_feedback[i]);
    ROS_INFO("Suppression%d %d", i, activity_suppression[i]);
    ROS_INFO("Impatience Reset%d %d", i, impatience_reset[i]);
    ROS_INFO("Acquiescence%d %d", i, acquiescence[i]);
    ROS_INFO("m1%d %0.2f", i, m1[i]);
  }

  behaviour_set();
}

void acquiescence_calc()
{

  if (active_time[current_task] > psi[current_task] &&
          comm[current_task][current_robot] == 1 ||
      active_time[current_task] > lambda[current_task])
  {
    ROS_INFO("Acquiescence_calc");
    acquiescence[current_task] = 0;
    impatience[current_task] = delta_slow[current_task];
  }
  else
  {
    acquiescence[current_task] = 1;
  }
  motivation_calc();
}

void impatience_calc()
{
  // ROS_INFO("Impatience_calc");
  if (comm[current_task][current_robot] == 1 &&
      comm_phi[current_task][current_robot] == 0)
  {
    impatience[current_task] = delta_slow[current_task];
  }
  else
  {
    impatience[current_task] = delta_fast[current_task];
  }
  acquiescence_calc();
}

void comm_received()
{
  // ROS_INFO("comm_received");
  ros::Time stamp = ros::Time::now();
  if (current_robot == id)
  {
    my_task = current_task;
    comm[current_task][current_robot] = 1;
    if (old_stamp[current_task][current_robot].isZero())
    {
      old_stamp[current_task][current_robot] = stamp;
      // j = j++;
    }
    // else
    //{
    diff_stamp[current_task][current_robot] =
        stamp - old_stamp[current_task][current_robot];
    active_time[current_task] +=
        diff_stamp[current_task][current_robot].toSec();
    // ROS_INFO("Tarefa atual e %d.", my_task);
    // ROS_INFO("Robô atual e %d.", current_robot);
    // ROS_INFO("diff_stamp atual e %d.",
    // diff_stamp[current_task][current_robot]);
    // ROS_INFO("old_stamp de %d.", old_stamp[current_task][current_robot]);
    // ROS_INFO("stamp %f.", stamp.toSec);
    // ROS_INFO("init_stamp %d.", init_stamp);
    ROS_INFO("Tempo de %d ativo e %d.", current_task,
             active_time[current_task]);
    //}
    old_stamp[current_task][current_robot] = stamp;
  }
  else
  {
    // task_counter[current_task][current_robot] =
    // task_counter[current_task][current_robot] + 1;

    if (old_stamp[current_task][current_robot].isZero())
    {
      old_stamp[current_task][current_robot] = stamp;
      // j = j++;
    }
    diff_stamp[current_task][current_robot] =
        stamp - old_stamp[current_task][current_robot];
    // Determinação do comm_received e do impatience_reset para valores dentro
    // do intervalo de tau1.
    if (diff_stamp[current_task][current_robot].toSec() < tau1)
    {
      comm[current_task][current_robot] = 1;
      impatience_reset[current_task] = 0;
    }
    else
    {
      comm[current_task][current_robot] = 0;
      impatience_reset[current_task] = 1;
    }
    // Determinação do comm_phi para valores de intervalo de comunicação entre
    // robôs maior que o tempo de conclusão de tareda phi.
    if ((stamp - init_stamp).toSec() < phi)
    {
      comm_phi[current_task][current_robot] = 0;
    }
    else if ((stamp - init_stamp).toSec() > phi)
    {
      comm_phi[current_task][current_robot] = 1;
    }
    old_stamp[current_task][current_robot] = stamp;
  }
  impatience_calc();
}

void taskCallback(const alliance_msgs::Message& taskmsg)
{
  // ROS_INFO("TaskCallback");
  current_task = taskmsg.task_id;
  current_robot = taskmsg.robot_id;
  task_done[1] = taskmsg.task_done1;
  task_done[2] = taskmsg.task_done2;
  task_done[3] = taskmsg.task_done3;
  comm_received();
}

void odomCallback(const nav_msgs::Odometry& odom_msg)
{
  current_posx = odom_msg.pose.pose.position.x;
  current_posy = odom_msg.pose.pose.position.y;
  float x = odom_msg.pose.pose.orientation.x;
  float y = odom_msg.pose.pose.orientation.y;
  float z = odom_msg.pose.pose.orientation.z;
  float w = odom_msg.pose.pose.orientation.w;

  th = atan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z);
  ROS_INFO("Theta: %f.", th * 180 / PI);

  odom_feedback = 1;
}

void sonarCallback(const sensor_msgs::PointCloud& cloud)
{
  sonar0 = sqrt(cloud.points[0].y * cloud.points[0].y);
  sonar1 = sqrt(cloud.points[1].x * cloud.points[1].x +
                cloud.points[1].y * cloud.points[1].y);
  sonar2 = sqrt(cloud.points[2].x * cloud.points[2].x +
                cloud.points[2].y * cloud.points[2].y);
  sonar3 = sqrt(cloud.points[3].x * cloud.points[3].x +
                cloud.points[3].y * cloud.points[3].y);
  sonar4 = sqrt(cloud.points[4].x * cloud.points[4].x +
                cloud.points[4].y * cloud.points[4].y);
  sonar5 = sqrt(cloud.points[5].y * cloud.points[5].y);
  sonar6 = sqrt(cloud.points[6].x * cloud.points[6].x +
                cloud.points[6].y * cloud.points[6].y);
  sonar7 = sqrt(cloud.points[7].x * cloud.points[7].x +
                cloud.points[7].y * cloud.points[7].y);
  ROS_INFO("%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f", sonar0, sonar1,
           sonar2, sonar3, sonar4, sonar5, sonar6, sonar7);
  range_feedback = 1;
}

int main(int argc, char** argv)
{
  ROS_INFO("Iniciando node alliance_robot0...");
  ros::init(argc, argv, "alliance_robot0");
  ros::NodeHandle n;
  if (init_stamp.isZero())
  {
    init_stamp = ros::Time::now();
  }
  ros::Publisher task_pub =
      n.advertise<alliance_msgs::Message>("TaskAllocation", 5);
  ros::Publisher cmd_vel_pub =
      n.advertise<geometry_msgs::Twist>("RosAria0/Robot0/cmd_vel", 1);
  ros::Publisher motivation_pub =
      n.advertise<alliance_msgs::Motivation>("Motivation_robot0", 1);
  ros::Publisher position_pub =
      n.advertise<geometry_msgs::PoseStamped>("Robot0_Position", 5);
  ros::Subscriber tasksub =
      n.subscribe<alliance_msgs::Message>("TaskAllocation", 5, taskCallback);
  ros::Subscriber odomsub =
      n.subscribe<nav_msgs::Odometry>("RosAria0/Robot0/odom", 10, odomCallback);
  ros::Subscriber sonarsub = n.subscribe<sensor_msgs::PointCloud>(
      "RosAria0/Robot0/sonar", 10, sonarCallback);

  ros::Rate loop_rate(rho1);
  while (ros::ok() && task_done[3] == 0)
  {
    switch (current_behaviour)
    {
    case 0:
      idle();
      break;
    case 1:
      wander();
      if (wander_count == 0)
      {
        d_wand = d;
        wander_count += 1;
      }
      if (d - d_wand >= 300.0)
      {
        vel_x = 0.0;
        vel_omega = 0.0;
        task_done[1] = 1;
        current_behaviour = 0;
      }
      break;
    case 2:
      boundary_overwatch();
      if (current_position.pose.position.x > 0.0 &&
          current_position.pose.position.y > 6.0 && pos_count < 1)
      {
        pos_wall_initx = current_position.pose.position.x;
        pos_wall_inity = current_position.pose.position.y;
        d_init = d;
        pos_count++;
        // ROS_INFO("Xi: %d,Yi: %d,Pos_count: %d, d_init =
        // %f",pos_wall_initx,pos_wall_inity,pos_count, d_init);
      }
      if (pos_count >= 1 and (d > (d_init + 50.0)))
      {
        cur_x = current_position.pose.position.x;
        cur_y = current_position.pose.position.y;
        if (cur_x == pos_wall_initx + 1.0 && cur_y == pos_wall_inity + 1.0 ||
            cur_x == pos_wall_initx - 1.0 && cur_y == pos_wall_inity - 1.0 ||
            cur_x == pos_wall_initx + 1.0 && cur_y == pos_wall_inity - 1.0 ||
            cur_x == pos_wall_initx - 1.0 && cur_y == pos_wall_inity + 1.0 ||
            cur_x == pos_wall_initx && cur_y == pos_wall_inity)
        {
          pos_count++;
        }
        // ROS_INFO("Xi: %d,Yi: %d,X: %d, Y: %d, Pos_count: %d, d_init =
        // %f",pos_wall_initx,pos_wall_inity,cur_x,cur_y, pos_count, d_init);
      }
      ROS_INFO("Xi: %d,Yi: %d,X: %d, Y: %d, Pos_count: %d, d_init = %f",
               pos_wall_initx, pos_wall_inity, cur_x, cur_y, pos_count, d_init);
      if (pos_count >= 2)
      {
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
        task_done[2] = 1;
        current_behaviour = 0;
      }
      break;
    case 3:
      report();
      if (task_done[1] == 1 and task_done[2] == 1 and report_progress > 1)
      {
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
        task_done[3] = 1;
        current_behaviour = 0;
        cmd_vel_pub.publish(vel);
      }
      break;
    }
    task_filler();
    task_pub.publish(taskmsg);
    cmd_vel_pub.publish(vel);
    motiv.header.stamp = ros::Time::now();
    motiv.task_id = 0;
    motiv.motivation = m1[0];
    motivation_pub.publish(motiv);
    motiv.header.stamp = ros::Time::now();
    motiv.task_id = 1;
    motiv.motivation = m1[1];
    motivation_pub.publish(motiv);
    motiv.header.stamp = ros::Time::now();
    motiv.task_id = 2;
    motiv.motivation = m1[2];
    motivation_pub.publish(motiv);
    motiv.header.stamp = ros::Time::now();
    motiv.task_id = 3;
    motiv.motivation = m1[3];
    motivation_pub.publish(motiv);
    /*for (int i = 0; i <=3; i++)
    {
         motiv.task_id = i;
         motiv.motivation = m1[i];

         motivation_pub.publish(motiv);
    }*/
    // Cálculo da posição atual do robô baseada na resposta da odometria
    current_position.header.frame_id = id;
    current_position.pose.position.x = current_posx + xi;
    current_position.pose.position.y = current_posy + yi;
    // Cálculo da distância percorrida pelo robô
    d += sqrt((current_posx - lastx) * current_posx - lastx +
              (current_posy - lasty) * current_posy - lasty);
    ROS_INFO("Distance traveled: %f.", d);
    lastx = current_posx;
    lasty = current_posy;
    // current_position.yaw = current_posth - thi;
    ROS_INFO(
        "A posicao calculada do robo e: x: %f, y: %f, Travelled distance: %f",
        current_position.pose.position.x, current_position.pose.position.y, d);
    position_pub.publish(current_position);

    sensory_feedback[1] = (1 - task_done[1]) * odom_feedback;
    sensory_feedback[2] = (1 - task_done[2]) * odom_feedback * range_feedback;
    sensory_feedback[3] = (1 - task_done[3]) * odom_feedback;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
