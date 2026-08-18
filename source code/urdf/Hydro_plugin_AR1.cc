/* Plugin for Hydro-dynamics Force
 * Contain Reactive Force and Resistant Force
 *
 * FOR AR=1
 *
 * Segments(0:s_num-2) are elliptic cylinder, and Segment(s_num-1) is plate
 *
 * Created by Donghao Li
 * Update time: March 23rd 2021
 *
 *
*/
#ifndef __GAZEBO_HYDRO_PLUGIN_HH__
#define __GAZEBO_HYDRO_PLUGIN_HH__

#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/gazebo.hh"
#include <cstdio>
#include <iomanip>
#include <numeric>
#include <cmath>
#include <string>

using std::endl;    using std::cout;  using std::fstream;   using std::FILE;    using std::ios; using std::to_string;
using std::fscanf;  using ignition::math::Vector3d;     using ignition::math::Pose3d;

namespace gazebo
{
class Hydro : public ModelPlugin
  {
    public: Hydro(){};

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        this->model = _model;
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&Hydro::OnUpdate, this));
    }

    public: virtual void Init(){};

    private: void OnUpdate()
    {
        common::Time currTime = this->model->GetWorld()->SimTime();
        common::Time stepTime = currTime - this->prevUpdateTime;
        this->prevUpdateTime = currTime;
        const double simu_time = this->model->GetWorld()->SimTime().Double();
        const double one_step = 0.00025;
        const double simu_step = simu_time*(1./one_step)*10.0;  const int record_step = int(4*(1./one_step)*10);
        const double reset_step= 0.1*(1./one_step)*10.0;
        const int NoA = 2;                                      const int s_num = NoA+2;

        const double Mass_head = 15.02E-3;        const double Mass_body = 9.346E-3;
        const double Mass_tail = 5.545E-3;        const double Mass_tip  = 1.735E-3;
        const double CoGMass = Mass_head+(s_num-3)*Mass_body+Mass_tail+Mass_tip;
    /////////////////////////////////////////////////////////////////////// Variables Declarantion
    ////////////////////////////////////// Coefficient
    /// fluid information
        const double pho_f = 1000.0;
    /// partial of added mass added in urdf, 0.0 means all in plugin
        const double Seg_M_total_ratio = 0.75;

    /////////////////////////////////////////////////////////////////// CK: Coefficient
    /// coefficient for fluid pressure force
        const double c_d = 1.2;                   const double c_f = 0.006;
    /// coefficient for added mass force
        const double Seg_Cm = 01.0;
    /// coefficient for fluid pressure force
        const double loco_indicator = 1.;
    ///////////////////////////////////////////////////////////////////

    //////////////////////////////// Added Mass Parameters
    /// Segment Parameter (Uniform)
        const double Info_M_total[4] = {29.47E-03, 18.33E-03,10.88E-03, 19.19E-03};  // int(M_0*ds)
        const double Info_M_S[4]     = {6.336E-04, 2.452E-04, 8.631E-05, 2.687E-04};  // int(M_0*s*ds)
        const double Info_M_0        =  6.853E-01;  // added mass per length
        const double Info_M_l        =  6.853E-01;  // added mass per length
        const double Info_cog[4]     = {21.75E-3,13.38E-3,7.934E-3,14.00E-3};  // Relative to s=0
        const double Info_lt[4]      = {43.00E-3,26.75E-3,19.75E-3,28.00E-3};  // Segment length
        const double Info_h_0        =  29.54E-3;  // Segment depth
        const double Info_IH0[4]     = {1.270E-03, 7.902E-04, 4.688E-04, 8.271E-04};
        const double Info_IH1[4]     = {2.731E-05, 1.057E-05, 3.720E-06, 1.158E-05};
        const double Info_IH2[4]     = {7.829E-07, 1.885E-07, 3.936E-08, 2.162E-07};
        const double Info_IH3[4]     = {2.525E-08, 3.781E-09, 4.684E-10, 4.539E-09};
        const double Info_IP[4]      = {3.092E-03, 1.924E-03, 1.141E-03, 1.772E-03};

        double Seg_M_total, Seg_M_S,  Seg_cog, Seg_lt, Seg_IH0, Seg_IH1, Seg_IH2, Seg_IH3, Seg_IP, Seg_lc;
        double Seg_M_0 = Info_M_0;
        double Seg_M_l = Info_M_l;
        double Seg_h_0 = Info_h_0;
     ////////////////////////// Hydro-dynamic Force related Variables
     /// wrench variables
        double react_x[10]={0.0};           double react_y[10]={0.0};           double react_zz[10]={0.0};
        double resis_x[10]={0.0};           double resis_y[10]={0.0};           double resis_t[10]={0.0};
        double fluip_x[2] ={0.0};
     /// absolute kinematic in mobile frame
        double vel_x[10]={0.0};     double vel_y[10]={0.0};    double vel_y_l[10]={0.0};
        double acc_x[10]={0.0};     double acc_y[10]={0.0};    double acc_y_ant[10]={0.0};
        double vel_yaw[10]={0.0};      double theta3[10]={0.0};
    /// Kinematic Variable of Link
        Pose3d WP[10];              // link position
        Vector3d WLV[10];           // pivot linear velocity in world frame
        Vector3d WCoGV[10];         // CoG linear velocity in world frame
        Vector3d WLA[10];           // pivot linear acceleration in world frame
        Vector3d WAV[10];           // angular velocity in world frame
        Vector3d WAA[10];           // angular acceleration in world frame
        Vector3d hydro_force[10];   // hydro-dynamic force in link frame
        Vector3d hydro_torque[10];  // hydro-dynamic torque in link frame
    /// Variables for calibration
        Vector3d cali_WLA; // linear acceleration in world
        Vector3d cali_WAA; // linear acceleration in world
    /// Variables for recording
        double rcra_x[10] = {0.0};     double rcra_y[10] = {0.0};     double rcra_z[10]={0.0};
        double  CoGVel_x  = 0.0;       double CoGVel_y   = 0.0;
        int trial=0;  int rollout=0;
    ////////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////// Dynamic Process
    ////////////////////// Calibration for redundant acceleration
        if(simu_step<=reset_step){
            for(int i=0;i<s_num;i++) {
                link[i] = this->model->GetLinks()[i];
                {
                    inertial[i] = link[i]->GetInertial();
                    cali_WLA = link[i]->WorldLinearAccel();
                    cali_WAA = link[i]->WorldAngularAccel();
                    link[i]->SetAngularVel(Vector3d(0.0,0.0,0.0));
                    link[i]->SetLinearVel(Vector3d(0.0,0.0,0.0));
                    link[i]->SetForce(Vector3d(-cali_WLA.X()*inertial[i]->Mass()*0.5,-cali_WLA.Y()*inertial[i]->Mass()*0.5,0.0));
                    link[i]->SetTorque(Vector3d(0.0,0.0,-cali_WAA.Z()*inertial[i]->IZZ()*0.5));
                }           // End of link[i]
            }               // End of i loop
        }                   // End of calibration
    ////////////////////// Calculation and applying for hydro-dynamic force
        else {
            for (int i=0;i<s_num;i++) {
                if (i==0){
                    Seg_M_total = Info_M_total[0];
                    Seg_M_S     = Info_M_S[0];
                    Seg_cog     = Info_cog[0];
                    Seg_lt      = Info_lt[0];
                    Seg_IH0     = Info_IH0[0];
                    Seg_IH1     = Info_IH1[0];
                    Seg_IH2     = Info_IH2[0];
                    Seg_IH3     = Info_IH3[0];
                    Seg_IP      = Info_IP[0];
                }
                else if (i==s_num-2) {
                    Seg_M_total = Info_M_total[2];
                    Seg_M_S     = Info_M_S[2];
                    Seg_cog     = Info_cog[2];
                    Seg_lt      = Info_lt[2];
                    Seg_IH0     = Info_IH0[2];
                    Seg_IH1     = Info_IH1[2];
                    Seg_IH2     = Info_IH2[2];
                    Seg_IH3     = Info_IH3[2];
                    Seg_IP      = Info_IP[2];
                }
                else if (i==s_num-1) {
                    Seg_M_total = Info_M_total[3];
                    Seg_M_S     = Info_M_S[3];
                    Seg_cog     = Info_cog[3];
                    Seg_lt      = Info_lt[3];
                    Seg_IH0     = Info_IH0[3];
                    Seg_IH1     = Info_IH1[3];
                    Seg_IH2     = Info_IH2[3];
                    Seg_IH3     = Info_IH3[3];
                    Seg_IP      = Info_IP[3];
                }
                else {
                    Seg_M_total = Info_M_total[1];
                    Seg_M_S     = Info_M_S[1];
                    Seg_cog     = Info_cog[1];
                    Seg_lt      = Info_lt[1];
                    Seg_IH0     = Info_IH0[1];
                    Seg_IH1     = Info_IH1[1];
                    Seg_IH2     = Info_IH2[1];
                    Seg_IH3     = Info_IH3[1];
                    Seg_IP      = Info_IP[1];
                }
                link[i] = this->model->GetLinks()[i];
                {

                    // get Rotation angle
                    WP[i]        = link[i]->WorldPose();
                    theta3[i]    = WP[i].Rot().Yaw();
                    // linear velocity transfer
                    WLV[i]       = link[i]->WorldLinearVel();                               // Velocity of Anterior Point in Global frame
                    vel_x[i]     = WLV[i].X()*cos(theta3[i])+WLV[i].Y()*sin(theta3[i]);     // Velocity of Anterior Point in Mobile frame
                    vel_y[i]     = WLV[i].Y()*cos(theta3[i])-WLV[i].X()*sin(theta3[i]);     // Velocity of Anterior Point in Mobile frame
                    WLA[i]       = link[i]->WorldLinearAccel();                             // Acceleration of CoG in Global frame
                    acc_x[i]     = WLA[i].X()*cos(theta3[i])+WLA[i].Y()*sin(theta3[i]);     // Acceleration of CoG in Mobile frame
                    acc_y[i]     =-WLA[i].X()*sin(theta3[i])+WLA[i].Y()*cos(theta3[i]); // Acceleration of CoG in Mobile frame
                    // angular velocity transfer
                    WAV[i]       = link[i]->WorldAngularVel();                              // Angular Velocity Vector
                    vel_yaw[i]   = WAV[i].Z();                                           // Yaw Angular Velocity
                    WAA[i]       = link[i]->WorldAngularAccel();                            // Yaw Angular Acceleration
                    vel_y_l[i]   = (vel_y[i]+vel_yaw[i]*Seg_lt);                            // Velocity of Post Point in Mobile frame
                    acc_y_ant[i] = acc_y[i]-Seg_cog*WAA[i].Z();                             // Velocity of Anterior Point in Mobile frame
                    // reactive force
                    react_x[i]   =  Seg_Cm * (
                                    +Seg_M_S     * pow(vel_yaw[i],2)
                                    +Seg_M_total * vel_y[i] * vel_yaw[i]
                                             )
                                    +Seg_M_total * Seg_M_total_ratio * acc_x[i];
                    react_y[i]   =  Seg_Cm * (
                                    +Seg_M_total * vel_x[i] * vel_yaw[i]
                                    -Seg_M_total * acc_y_ant[i]
                                    -Seg_M_S     * WAA[i].Z()
                                    -Seg_M_l     * vel_x[i] * vel_y_l[i]
                                    +Seg_M_0     * vel_x[i] * vel_y[i]
                                             )
                                    +Seg_M_total * Seg_M_total_ratio * acc_y[i];
                    react_zz[i]  = -Seg_Cm * (
                                    +Seg_M_l     * Seg_lt * vel_x[i] * vel_y_l[i]
                                    +Seg_M_S     * acc_y_ant[i]
                                    -Seg_M_S     * vel_x[i] * vel_yaw[i]
                                             )
                                    +Seg_M_total * Seg_M_total_ratio * acc_y[i] * Seg_cog;


                    if(i==0){
                        fluip_x[0]  = +0.5 * loco_indicator * Seg_M_0 * pow(vel_y[0],2);
                        react_x[i] += fluip_x[0];
                    }
                    else if (i==s_num-1) {
                        fluip_x[1]  = -0.5 * loco_indicator * Seg_M_l * pow(vel_y_l[s_num-1],2);
                        react_x[i] += fluip_x[1];
                    }

                    rcra_x[i] = Seg_Cm * (
                                +Seg_M_S     * pow(vel_yaw[i],2)
                                +Seg_M_total * vel_y[i] * vel_yaw[i]
                                         );
                    rcra_y[i] = Seg_Cm * (
                                +Seg_M_total * vel_x[i] * vel_yaw[i]
                                -Seg_M_total * acc_y_ant[i]
                                -Seg_M_S     * WAA[i].Z()
                                -Seg_M_l     * vel_x[i] * vel_y_l[i]
                                +Seg_M_0     * vel_x[i] * vel_y[i]
                                         );
                    rcra_z[i] =-Seg_Cm * (
                                +Seg_M_l     * Seg_lt * vel_x[i] * vel_y_l[i]
                                +Seg_M_S     * acc_y_ant[i]
                                -Seg_M_S     * vel_x[i] * vel_yaw[i]
                                         );

                    // Resis Force
                    resis_x[i] = 0.0;   resis_y[i] = 0.0;   resis_t[i] = 0.0;

                    if(vel_yaw[i]>=1E-8 || vel_yaw[i]<=-1E-8){
                        Seg_lc = -vel_y[i]/vel_yaw[i];
                    }
                    else {
                        Seg_lc = 0.0;
                    }
                    // Friction Calculation
                    resis_x[i] = (-0.5*pho_f*c_f)*fabs(vel_x[i])*vel_x[i]*Seg_IP;
                    // Drag Calculation
                    if(vel_yaw[i]<=1E-8 && vel_yaw[i]>=-1E-8){
                        resis_y[i] = (-0.5*pho_f*c_d)*fabs(vel_y[i])*vel_y[i]*Seg_IH0;
                        resis_t[i] = (-0.5*pho_f*c_d)*fabs(vel_y[i])*vel_y[i]*Seg_IH1;
                    }
                    else if(vel_y[i]<=1E-8 && vel_y[i]>=-1E-8){
                        resis_y[i] = (-0.5*pho_f*c_d)*fabs(vel_yaw[i])*vel_yaw[i]*Seg_IH2;
                        resis_t[i] = (-0.5*pho_f*c_d)*fabs(vel_yaw[i])*vel_yaw[i]*Seg_IH3;
                    }
                    else if (Seg_lc<0 || Seg_lc>=Seg_lt ) {
                        resis_y[i]= (-0.5*pho_f*c_d) * (fabs(vel_y[i])/vel_y[i])
                                *(
                                   +pow(vel_y[i],2)           * Seg_IH0
                                   +2*vel_yaw[i]*vel_y[i]     * Seg_IH1
                                   +pow(vel_yaw[i],2)         * Seg_IH2
                                  );
                        resis_t[i]= (-0.5*pho_f*c_d) * (fabs(vel_y[i])/vel_y[i])
                                *(
                                   +pow(vel_y[i],2)           * Seg_IH1
                                   +2*vel_yaw[i]*vel_y[i]     * Seg_IH2
                                   +pow(vel_yaw[i],2)         * Seg_IH3
                                  );
                    }
                    else {
                        resis_y[i]=(-0.5*pho_f*c_d*Seg_h_0) * (fabs(vel_y[i])/vel_y[i])
                                *(
                                   +pow(vel_y[i],2)     * (2.0*Seg_lc-Seg_lt)
                                   +vel_y[i]*vel_yaw[i] * (2.0*pow(Seg_lc,2)-pow(Seg_lt,2))
                                   +pow(vel_yaw[i],2)   * (2./3.*pow(Seg_lc,3)-1.0/3.0*pow(Seg_lt,3))
                                  );
                        resis_t[i]=(-0.5*pho_f*c_d*Seg_h_0)*(fabs(vel_y[i])/vel_y[i])
                                *(
                                   +pow(vel_y[i],2)     * (1.0*pow(Seg_lc,2)-0.5*pow(Seg_lt,2))
                                   +vel_y[i]*vel_yaw[i] * (4.0/3.0*pow(Seg_lc,3)-2.0/3.0*pow(Seg_lt,3))
                                   +pow(vel_yaw[i],2)   * (0.5*pow(Seg_lc,4)-1.0/4.0*pow(Seg_lt,4))
                                  );
                    }
                    link[i]->AddLinkForce( Vector3d(react_x[i]+resis_x[i], react_y[i]+resis_y[i], 0.0) );
                    link[i]->AddTorque( Vector3d(0.0,  0.0,  react_zz[i]+resis_t[i]) );


                  }         // End of link[i]
              }             // End of i loop
            }               // End of applying force
    //////////////////////////////////////////////////////////////////////////////////// Recording Process
    ///  Velocity after t=6.0s (Reward)
        fstream reward_velocity;
        if (simu_step == record_step){
            reward_velocity.open("/home/donghao/result/DD/velocity.csv",ios::out); reward_velocity.close();
        }
        if (simu_step > record_step){
            reward_velocity.open("/home/donghao/result/DD/velocity.csv",ios::app);
            for(int i=0;i<s_num;i++) {
                link[i] = this->model->GetLinks()[i];
                {
                    WCoGV[i] = link[i]->WorldCoGLinearVel();
                    if(i==s_num-1){
                        CoGVel_x += Mass_tip  * WCoGV[i].X();
                        CoGVel_y += Mass_tip  * WCoGV[i].Y();
                    }
                    else if (i==s_num-2) {
                        CoGVel_x += Mass_tail * WCoGV[i].X();
                        CoGVel_y += Mass_tail * WCoGV[i].Y();
                    }
                    else if (i==0) {
                        CoGVel_x += Mass_head * WCoGV[i].X();
                        CoGVel_y += Mass_head * WCoGV[i].Y();
                    }
                    else {
                        CoGVel_x += Mass_body * WCoGV[i].X();
                        CoGVel_y += Mass_body * WCoGV[i].Y();
                    }
                }   // End of link[i]
            }       // End of i loop
            CoGVel_x = CoGVel_x/CoGMass;
            CoGVel_y = CoGVel_y/CoGMass;
            reward_velocity<<CoGVel_x<<','<<CoGVel_y<<endl;    reward_velocity.close();
        }
        ///Data recorded for gait analysis
        FILE *Inputfile;
        Inputfile=fopen("/home/donghao/result/DD/indicator.csv","r");
        fscanf(Inputfile, "%d %d",&trial,&rollout);   fclose(Inputfile);

        if (trial>=1 && rollout<3){
            fstream joint_record;            fstream links_record;             fstream force_record;     fstream kinet_record;      fstream link2_record;

            if (simu_step == record_step){
                joint_record.open("/home/donghao/result/data/"+to_string(trial)+"/"+to_string(rollout)+"_joint_record.csv",ios::out); joint_record.close();
                links_record.open("/home/donghao/result/data/"+to_string(trial)+"/"+to_string(rollout)+"_links_record.csv",ios::out); links_record.close();
                force_record.open("/home/donghao/result/data/"+to_string(trial)+"/"+to_string(rollout)+"_force_record.csv",ios::out); force_record.close();
                kinet_record.open("/home/donghao/result/data/"+to_string(trial)+"/"+to_string(rollout)+"_kinet_record.csv",ios::out); kinet_record.close();
            }
            else if (simu_step>record_step) {
                joint_record.open("/home/donghao/result/data/"+to_string(trial)+"/"+to_string(rollout)+"_joint_record.csv",ios::app);
                links_record.open("/home/donghao/result/data/"+to_string(trial)+"/"+to_string(rollout)+"_links_record.csv",ios::app);
                force_record.open("/home/donghao/result/data/"+to_string(trial)+"/"+to_string(rollout)+"_force_record.csv",ios::app);
                kinet_record.open("/home/donghao/result/data/"+to_string(trial)+"/"+to_string(rollout)+"_kinet_record.csv",ios::app);


                /// Joint Recording: Position, Velocity
                joint_record<<simu_time;
                for(int i=0;i<s_num-1;i++) {
                    joint[i] = this->model->GetJoints()[i];
                    {
                        joint_record<<','<<joint[i]->Position(0)<<','<<joint[i]->GetVelocity(0);
                    }
                }   // End of loop
                joint_record<<endl;     joint_record.close();
                /// Link Recording: Px,Py,Yaw;      Force Recording: Px,Py,Yaw;     Kinetic Recording:
                links_record<<simu_time;
                force_record<<simu_time;
                kinet_record<<simu_time;
                for(int i=0;i<s_num;i++) {
                    links_record<<','<<WP[i].Pos().X()<<','<<WP[i].Pos().Y()<<','<<WP[i].Rot().Yaw();
                    force_record<<','<<rcra_x[i]<<','<<rcra_y[i]<<','<<rcra_z[i]<<','<<resis_x[i]<<','<<resis_y[i]<<','<<resis_t[i];
                    kinet_record<<','<<WLV[i].X()<<','<<WLV[i].Y()<<','<<WAV[i].Z()<<','<<WLA[i].X()<<','<<WLA[i].Y()<<','<<WAA[i].Z();
                }
                links_record<<','<<WP[s_num-1].Pos().Y()+Info_lt[3]*sin(WP[s_num-1].Rot().Yaw() )<<endl;                links_record.close();
                force_record<<','<<fluip_x[0]<<','<<fluip_x[1]<<endl;                                                   force_record.close();
                kinet_record<<endl;                                                                                     kinet_record.close();
            }
        }
               // End of Recording
    ////////////////////////////////////////////////////////////////////////////////////
      }             // End of OnUpdate
    private: physics::LinkPtr link[10];

    private: physics::InertialPtr inertial[10];

    private: physics::JointPtr joint[10];

    private: event::ConnectionPtr updateConnection;

    private: physics::ModelPtr model;

    private: common::Time prevUpdateTime;

  };
  GZ_REGISTER_MODEL_PLUGIN(Hydro)
}
#endif
