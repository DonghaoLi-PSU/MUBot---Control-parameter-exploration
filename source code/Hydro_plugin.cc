/* Plugin for Hydro-dynamics Force
 * Contain Reactive Force and Resistant Force
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
    /////////////////////////////////////////////////////////////////////
    // Structural constants: these describe the fixed shape of the model
    // (segment count, number of distinct segment "types" the per-segment
    // tables are indexed by) and can never change at runtime
    private: static constexpr int NoA = 2;
    private: static constexpr int s_num = NoA + 2;
    private: static constexpr int SEGMENT_TYPES = 4; // head, body, tail, tip
    private: static constexpr double pho_f = 1000.0;
    private: static constexpr double Seg_M_total_ratio = 0.75;
    public: Hydro(){};

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
        this->model = _model;
        this->resultPath = "./result";
        this->dataPath   = "./result/data";
        this->paramPath  = this->resultPath + "/DD/hydro_parameter.csv";
        if (_sdf && _sdf->HasElement("result_path"))
            this->resultPath = _sdf->Get<std::string>("result_path");
        if (_sdf && _sdf->HasElement("data_path"))
            this->dataPath = _sdf->Get<std::string>("data_path");
        if (_sdf && _sdf->HasElement("param_file"))
            this->paramPath = _sdf->Get<std::string>("param_file");
        else if (_sdf && _sdf->HasElement("result_path"))
            this->paramPath = this->resultPath + "/DD/hydro_parameter.csv";

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&Hydro::OnUpdate, this));
    }

    public: virtual void Init() override
    {
        // Physical parameters: read once from hydro_parameter.csv. Falls
        // back to the built-in defaults already set as member initializers
        // if the file is missing or a field can't be parsed.
        this->LoadHydroParameters();
        this->CoGMass = Mass_head + (s_num - 3) * Mass_body + Mass_tail + Mass_tip;

        // Cache link pointers once. Calling this->model->GetLinks()[i] on
        // every physics step is wasteful; the link set is fixed after Load.
        for (int i = 0; i < s_num; i++)
        {
            this->link[i] = this->model->GetLinks()[i];
            this->inertial[i] = this->link[i]->GetInertial();
        }
        for (int i = 0; i < s_num - 1; i++)
        {
            this->joint[i] = this->model->GetJoints()[i];
        }
    };

    /////////////////////////////////////////////////////////////////////
    // Parses hydro_parameter.csv. Expected format: one parameter per
    // line, comma separated, `#` starts a comment line.
    //  AR05
    //   Mass_head,0.06010
    //   Mass_body,0.03739
    //   Mass_tail,0.02218
    //   Mass_tip,0.006939
    //   c_d,2.25
    //   c_f,0.06
    //   Seg_Cm,1.0
    //   loco_indicator,1.0
    //   Info_M_0,2.741
    //   Info_M_l,2.741
    //   Info_h_0,0.05908
    //   Info_M_total,0.1179,0.07333,0.04351,0.07676
    //   Info_M_S,0.002534,0.0009808,0.0003452,0.001075
    //   Info_cog,0.02175,0.01338,0.007934,0.01400
    //   Info_lt,0.04300,0.02675,0.01975,0.02800
    //   Info_IH0,0.002540,0.001580,0.0009376,0.001654
    //   Info_IH1,0.00005462,0.00002114,0.00000744,0.00002316
    //   Info_IH2,0.000001566,0.0000003770,0.00000007871,0.0000004323
    //   Info_IH3,0.0000000505,0.000000007563,0.0000000009369,0.000000009079
    //   Info_IP,0.006185,0.003847,0.002283,0.003543

    private: void LoadHydroParameters()
    {
        std::ifstream file(this->paramPath);
        if (!file.is_open())
        {
            gzwarn << "[Hydro] Could not open parameter file '" << this->paramPath
                   << "', using built-in default hydrodynamic parameters." << std::endl;
            return;
        }

        std::string line;
        while (std::getline(file, line))
        {
            // Strip trailing carriage return / whitespace, skip blanks & comments.
            while (!line.empty() && (line.back()=='\r' || line.back()==' ' || line.back()=='\t'))
                line.pop_back();
            if (line.empty() || line[0] == '#')
                continue;

            std::stringstream ss(line);
            std::string key;
            std::getline(ss, key, ',');

            std::vector<double> vals;
            std::string tok;
            while (std::getline(ss, tok, ','))
            {
                try { vals.push_back(std::stod(tok)); }
                catch (...) { /* skip unparsable token */ }
            }
            if (vals.empty())
                continue;

            this->AssignParameter(key, vals);
        }
    }

    private: void AssignParameter(const std::string &key, const std::vector<double> &v)
    {
        auto assign4 = [&](double (&arr)[4]) {
            if (v.size() < SEGMENT_TYPES)
            {
                gzwarn << "[Hydro] Parameter '" << key << "' needs " << SEGMENT_TYPES
                       << " values, got " << v.size() << "; keeping default." << std::endl;
                return;
            }
            for (int i = 0; i < SEGMENT_TYPES; i++) arr[i] = v[i];
        };

        if      (key == "Mass_head")            this->Mass_head = v[0];
        else if (key == "Mass_body")             this->Mass_body = v[0];
        else if (key == "Mass_tail")             this->Mass_tail = v[0];
        else if (key == "Mass_tip")              this->Mass_tip = v[0];
        else if (key == "c_d")                   this->c_d = v[0];
        else if (key == "c_f")                   this->c_f = v[0];
        else if (key == "Seg_Cm")                this->Seg_Cm = v[0];
        else if (key == "loco_indicator")        this->loco_indicator = v[0];
        else if (key == "Info_M_0")               { this->Info_M_0 = v[0]; }
        else if (key == "Info_M_l")               { this->Info_M_l = v[0]; }
        else if (key == "Info_h_0")               { this->Info_h_0 = v[0]; }
        else if (key == "Info_M_total")          assign4(this->Info_M_total);
        else if (key == "Info_M_S")               assign4(this->Info_M_S);
        else if (key == "Info_cog")               assign4(this->Info_cog);
        else if (key == "Info_lt")                assign4(this->Info_lt);
        else if (key == "Info_IH0")                assign4(this->Info_IH0);
        else if (key == "Info_IH1")                assign4(this->Info_IH1);
        else if (key == "Info_IH2")                assign4(this->Info_IH2);
        else if (key == "Info_IH3")                assign4(this->Info_IH3);
        else if (key == "Info_IP")                 assign4(this->Info_IP);
        else
            gzwarn << "[Hydro] Unknown parameter '" << key << "' in "
                   << this->paramPath << "; ignoring." << std::endl;
    }

    private: void OnUpdate()
    {
        common::Time currTime = this->model->GetWorld()->SimTime();
        common::Time stepTime = currTime - this->prevUpdateTime;
        this->prevUpdateTime = currTime;
        const double simu_time = this->model->GetWorld()->SimTime().Double();
        const double one_step = 0.00025;
        const double simu_step = simu_time*(1./one_step)*10.0;  const int record_step = int(4*(1./one_step)*10);
        const double reset_step= 0.1*(1./one_step)*10.0;
    /////////////////////////////////////////////////////////////////////// Variables Declarantion
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
        const std::string velocityFile = this->resultPath + "/DD/velocity.csv";
        if (simu_step == record_step){
            reward_velocity.open(velocityFile,ios::out); reward_velocity.close();
        }
        if (simu_step > record_step){
            reward_velocity.open(velocityFile,ios::app);
            for(int i=0;i<s_num;i++) {
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
         ////////////////////////////////////////////////////////////////////
        /// Trial/rollout indicator: only poll the file periodically 

        static const int INDICATOR_POLL_INTERVAL = 40; // ~10ms @ one_step=0.00025
        if (this->updateCounter % INDICATOR_POLL_INTERVAL == 0 || simu_step <= reset_step)
        {
            const std::string indicatorFile = this->resultPath + "/DD/indicator.csv";
            FILE *Inputfile = fopen(indicatorFile.c_str(), "r");
            if (Inputfile != nullptr)
            {
                int t = this->cachedTrial, r = this->cachedRollout;
                if (fscanf(Inputfile, "%d %d", &t, &r) == 2)
                {
                    this->cachedTrial = t;
                    this->cachedRollout = r;
                }
                fclose(Inputfile);
            }
            // If the file is temporarily unavailable, keep using the last
            // known good trial/rollout values instead of crashing or
            // silently resetting to zero.
        }
        this->updateCounter++;
        const int trial = this->cachedTrial;
        const int rollout = this->cachedRollout;

            fstream joint_record;            fstream links_record;             fstream force_record;     fstream kinet_record; 
            const std::string trialDir = this->dataPath + "/" + to_string(trial) + "/" + to_string(rollout);

            if (simu_step == record_step){
                joint_record.open(trialDir+"_joint_record.csv",ios::out); joint_record.close();
                links_record.open(trialDir+"_links_record.csv",ios::out); links_record.close();
                force_record.open(trialDir+"_force_record.csv",ios::out); force_record.close();
                kinet_record.open(trialDir+"_kinet_record.csv",ios::out); kinet_record.close();
            }
            else if (simu_step>record_step) {
                joint_record.open(trialDir+"_joint_record.csv",ios::app);
                links_record.open(trialDir+"_links_record.csv",ios::app);
                force_record.open(trialDir+"_force_record.csv",ios::app);
                kinet_record.open(trialDir+"_kinet_record.csv",ios::app);


                /// Joint Recording: Position, Velocity
                if (joint_record.is_open())
                {
                    joint_record<<simu_time;
                    for(int i=0;i<s_num-1;i++) {
                        joint_record<<','<<joint[i]->Position(0)<<','<<joint[i]->GetVelocity(0);
                    }   // End of loop
                    joint_record<<endl;
                }
                joint_record.close();
                /// Link Recording: Px,Py,Yaw;      Force Recording: Px,Py,Yaw;     Kinetic Recording:
                if (links_record.is_open() && force_record.is_open() && kinet_record.is_open())
                {
                    links_record<<simu_time;
                    force_record<<simu_time;
                    kinet_record<<simu_time;
                    for(int i=0;i<s_num;i++) {
                        links_record<<','<<WP[i].Pos().X()<<','<<WP[i].Pos().Y()<<','<<WP[i].Rot().Yaw();
                        force_record<<','<<rcra_x[i]<<','<<rcra_y[i]<<','<<rcra_z[i]<<','<<resis_x[i]<<','<<resis_y[i]<<','<<resis_t[i];
                        kinet_record<<','<<WLV[i].X()<<','<<WLV[i].Y()<<','<<WAV[i].Z()<<','<<WLA[i].X()<<','<<WLA[i].Y()<<','<<WAA[i].Z();
                    }
                    links_record<<','<<WP[s_num-1].Pos().Y()+Info_lt[3]*sin(WP[s_num-1].Rot().Yaw() )<<endl;
                    force_record<<','<<fluip_x[0]<<','<<fluip_x[1]<<endl;
                    kinet_record<<endl;
                }
                links_record.close(); force_record.close(); kinet_record.close();
            
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

    private: std::string resultPath;

    private: std::string dataPath;

    private: std::string paramPath;

    private: int cachedTrial = 0;

    private: int cachedRollout = 0;

    private: unsigned long updateCounter = 0;

    /////////////////////////////////////////////////////////////////////
    // Physical parameters. These are set to the model's original
    // values as defaults, then overwritten (per-field) by
    // LoadHydroParameters() if hydro_parameter.csv provides them.
    private: double Mass_head = 60.10E-03;
    private: double Mass_body = 37.39E-03;
    private: double Mass_tail = 22.18E-03;
    private: double Mass_tip  = 6.939E-03;
    private: double CoGMass   = 0.0; // computed in Init() after masses are loaded

    private: double pho_f = 1000.0;
    private: double Seg_M_total_ratio = 0.75;

    private: double c_d = 2.25;
    private: double c_f = 0.06;
    private: double Seg_Cm = 1.0;
    private: double loco_indicator = 1.0;

    private: double Info_M_total[4] = {1.179E-01, 7.333E-02, 4.351E-02, 7.676E-02};  // int(M_0*ds)
    private: double Info_M_S[4]     = {2.534E-03, 9.808E-04, 3.452E-04, 1.075E-03};  // int(M_0*s*ds)
    private: double Info_M_0        =  2.741;  // added mass per length
    private: double Info_M_l        =  2.741;  // added mass per length
    private: double Info_cog[4]     = {21.75E-3,13.38E-3,7.934E-3,14.00E-3};  // Relative to s=0
    private: double Info_lt[4]      = {43.00E-3,26.75E-3,19.75E-3,28.00E-3};  // Segment length
    private: double Info_h_0        =  59.08E-3;  // Segment depth
    private: double Info_IH0[4]     = {2.540E-03, 1.580E-03, 9.376E-04, 1.654E-03};
    private: double Info_IH1[4]     = {5.462E-05, 2.114E-05, 7.440E-06, 2.316E-05};
    private: double Info_IH2[4]     = {1.566E-06, 3.770E-07, 7.871E-08, 4.323E-07};
    private: double Info_IH3[4]     = {5.050E-08, 7.563E-09, 9.369E-10, 9.079E-09};
    private: double Info_IP[4]      = {6.185E-03, 3.847E-03, 2.283E-03, 3.543E-03};

  };
  GZ_REGISTER_MODEL_PLUGIN(Hydro)
}
#endif
