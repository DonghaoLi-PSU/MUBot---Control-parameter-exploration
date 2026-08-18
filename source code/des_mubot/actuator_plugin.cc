/* Plugin for Megnetic-electric Actuator & Spring
 *
 * Joints(0:s_num-3) contain active actuator and passive spring
 * Joint (s_num-2)   contain passive spring
 *
 * Created by Donghao Li
 * Update time: March 23rd 2021
*/
#ifndef __GAZEBO_ACTUATOR_PLUGIN_HH__
#define __GAZEBO_ACTUATOR_PLUGIN_HH__

#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/gazebo.hh"
#include <array>
#include <cstdio>
#include <math.h>
#include <iomanip>
#include <sstream>
#include <string>
#define PI 3.141592653589793238463

using std::endl;    using std::cout;  using std::fstream;   using std::FILE;    using std::ios; using std::fscanf;
using std::string;  using std::stringstream;    using std::ifstream;    using std::to_string;

namespace gazebo
{
    class ActuatorPlugin : public ModelPlugin
    {
        public: ActuatorPlugin(){};

        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
        {
            this->model = _model;

            // Configurable output/input locations (defaults preserve
            // original hardcoded behaviour).
            this->resultPath = "./result";
            this->dataPath   = "./result/data";
            if (_sdf && _sdf->HasElement("result_path"))
                this->resultPath = _sdf->Get<std::string>("result_path");
            if (_sdf && _sdf->HasElement("data_path"))
                this->dataPath = _sdf->Get<std::string>("data_path");

            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&ActuatorPlugin::OnUpdate, this));
        };

        public: virtual void Init() override
        {
            // Cache joint pointers once instead of re-resolving them from
            // this->model->GetJoints()[i] on every physics tick.
            const int NoA = 2;
            const int s_num = NoA + 2;
            for (int i = 0; i < s_num - 1; i++)
            {
                this->joint[i] = this->model->GetJoints()[i];
            }
        };

        private: void OnUpdate()
        {
            common::Time currTime = this->model->GetWorld()->SimTime();
            this->prevUpdateTime = currTime;
            const double simu_time = this->model->GetWorld()->SimTime().Double();
            const double one_step = 0.00025;
            const double simu_step = simu_time*(1./one_step)*10.0;  const int record_step = int(4*(1./one_step)*10);
            const int NoA = 2;                                      const int s_num = NoA+2;


            ///////////////////////////////////////////// Variables Declarantion
            double  pos[10] = {0.0};
            double  torque_actuator[10] = {0.0};    double  torque_spring[10] = {0.0};
            const double ac_ratio = 10.0;     const double stiff_ratio = 5.;
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            static const int INDICATOR_POLL_INTERVAL = 40; // ~10ms @ one_step=0.00025
            if (this->updateCounter % INDICATOR_POLL_INTERVAL == 0)
            {
                const std::string indicatorFile = this->resultPath + "/DD/indicator.csv";
                FILE *IndicateFile = fopen(indicatorFile.c_str(), "r");
                if (IndicateFile != nullptr)
                {
                    int t = this->cachedTrial, r = this->cachedRollout;
                    if (fscanf(IndicateFile, "%d %d", &t, &r) == 2)
                    {
                        if (t != this->cachedTrial || r != this->cachedRollout)
                            this->inputNeedsReload = true;   // new rollout -> refresh cached command
                        this->cachedTrial = t;
                        this->cachedRollout = r;
                    }
                    fclose(IndicateFile);
                }
            }
            this->updateCounter++;
            const int trial = this->cachedTrial;
            const int rollout = this->cachedRollout;

            ///////////////////////////////////////////// Dynamic Process
            if(simu_step>1000){
                if (this->inputNeedsReload)
                {
                    const std::string inputFile = this->resultPath + "/DD/joint_input.csv";
                    ifstream Inputfile(inputFile, ios::in);
                    if (Inputfile.is_open())
                    {
                        string inputline, colo;
                        if (std::getline(Inputfile, inputline))
                        {
                            stringstream iss(inputline);
                            // NOTE: this parses exactly 2*s_num-3 values, sized for
                            // NoA=2 (s_num=4). If NoA changes, this loop bound and
                            // the joint_raw indices below must be re-derived.
                            for (int i=0;i<2*s_num-3;i++) {
                                if (!std::getline(iss,colo,',')) break;
                                stringstream convertor(colo);
                                convertor >> this->joint_raw[i];
                            }
                            this->joint_input[0] = 0.0;
                            for(int i=0;i<s_num-2;i++){
                                this->joint_input[2*i+1] = fabs(this->joint_raw[2*i]);
                                this->joint_input[2*i+2] =      this->joint_raw[2*i+1];
                            }
                            this->frequency    = fabs(this->joint_raw[s_num*2-5]);
                            this->spring_stiff = fabs(this->joint_raw[s_num*2-4]);
                            this->inputNeedsReload = false;
                        }
                    }
                    // If the file couldn't be read this tick, keep using the
                    // previously cached command and try again next poll.
                }

                /// Applying Torque for actuators
                for (int i=0;i<s_num-2;i++) {
                    {
                        pos[i] = joint[i]->Position(0);
                        double vel = joint[i]->GetVelocity(0);
                        double voltage = this->joint_input[2*i+1]*sin(2*PI*(this->frequency*simu_time+this->joint_input[2*i]));
                        if(voltage>15.0){
                            voltage=15.0;
                        }
                        else if (voltage<(-15.0)) {
                            voltage=-15.0;
                        };
                        torque_actuator[i] = (voltage-0.009079*vel)/90*0.009079*ac_ratio;
                        torque_spring[i] = -this->spring_stiff*pos[i]*ac_ratio;
                        joint[i]->SetForce(0, torque_spring[i]+torque_actuator[i]);
                    };
                };           // End of loop
                /// Applying Torque for passive joint
                {
                    pos[s_num-2] = joint[s_num-2]->Position(0);
                    torque_spring[s_num-2] = (- stiff_ratio*this->spring_stiff*pos[s_num-2]) *ac_ratio;
                    joint[s_num-2]->SetForce(0, torque_spring[s_num-2]);
                };
            };
            ////////////////////////////////////////////////////////////////////////////////////////////////////////

             ///////////////////////////////////////////// Recording
             if (trial>=1 && rollout==0){
                 fstream hydro_param_record;
                 if (simu_step == record_step){
                     const std::string paramFile = this->dataPath + "/" + to_string(trial) + "/actuator_record.csv";
                     hydro_param_record.open(paramFile,ios::out);
                     if (hydro_param_record.is_open())
                        hydro_param_record<< this->spring_stiff <<','<< stiff_ratio <<','<<ac_ratio<<','<< s_num<<endl;
                     hydro_param_record.close();
                 }
             }
            // /// Actuator parameter recording
             if (trial>=1 && simu_step == record_step  && rollout<3){
                 fstream actuator_param;
                 const std::string paramFile = this->dataPath + "/" + to_string(trial) + "/" + to_string(rollout) + "_actuator_param.csv";
                 actuator_param.open(paramFile,ios::out);
                 if (actuator_param.is_open())
                 {
                     for (int i=0;i<s_num*2-4;i++) {
                         actuator_param<<this->joint_input[i]<<',';
                     }
                     actuator_param<<this->frequency<<','<<this->spring_stiff<<','<<stiff_ratio<<endl;
                 }
                 actuator_param.close();
                 }
             /// Actuator parameter recording
             if (trial>=1 && simu_step >= record_step && rollout<3){
                 fstream actuator_output;
                 const std::string outputFile = this->dataPath + "/" + to_string(trial) + "/" + to_string(rollout) + "_actuator_output.csv";
                 if(simu_step == record_step){
                      actuator_output.open(outputFile,ios::out);
                 }
                 else {
                      actuator_output.open(outputFile,ios::app);
                      if (actuator_output.is_open())
                      {
                          actuator_output<<simu_time;
                          for(int i=0;i<s_num-1;i++){
                              actuator_output<<','<<torque_actuator[i]<<','<<torque_spring[i];
                          }
                          actuator_output<<endl;
                      }
                 }
                 actuator_output.close();
             }
        //////////////////////////////////////////////////////////////////////////////////////////////////////////
        };       // End of OnUpdate

        private: physics::JointPtr joint[10];

        private: event::ConnectionPtr updateConnection;

        private: physics::ModelPtr model;

        private: common::Time prevUpdateTime;

        private: std::string resultPath;

        private: std::string dataPath;

        private: int cachedTrial = 0;

        private: int cachedRollout = -1;

        private: unsigned long updateCounter = 0;

        // Cached rollout command, refreshed only when the trial/rollout changes.
        private: bool inputNeedsReload = true;

        private: double joint_raw[30] = {0.0};

        private: double joint_input[30] = {0.0};

        private: double frequency = 0.0;

        private: double spring_stiff = 0.0;

    };
    GZ_REGISTER_MODEL_PLUGIN(ActuatorPlugin)
}
#endif