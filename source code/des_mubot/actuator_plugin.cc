/* Plugin for Megnetic-electric Actuator & Spring
 *
 * Joints(0:s_num-3) contain active actuator and passive spring
 * Joint (s_num-2)   contain passive spring
 *
 * Created by Donghao Li
 * Update time: March 23rd 2021
 *
 *
*/
#ifndef __GAZEBO_ACTUATOR_PLUGIN_HH__
#define __GAZEBO_ACTUATOR_PLUGIN_HH__

#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/gazebo.hh"
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
    
        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            this->model = _model;
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&ActuatorPlugin::OnUpdate, this));
        };
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
        
        
            ///////////////////////////////////////////// Variables Declarantion
            double  pos[10] = {0.0};        double  vel[10] = {0.0};
            double  voltage[10] = {0.0};    double  torque_actuator[10] = {0.0};    double  torque_spring[10] = {0.0};
            double  joint_raw[30] = {0.0};  double  joint_input[30] = {0.0};        double frequency = 0.0;
        //    const double spring_stiff = 7.5E-3;
            double spring_stiff = 0.0;
            ifstream Inputfile;    string inputline;  string colo;
            int trial=0;  int rollout=0;
            fstream actuator_param;         fstream actuator_output;
            const double ac_ratio = 10.0;     const double stiff_ratio = 5.;
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////        
            ///////////////////////////////////////////// Dynamic Process
            if(simu_step>1000){
                /// Read actuator input from csv
                Inputfile.open("/home/donghao/result/DD/joint_input.csv",ios::in);
                std::getline(Inputfile,inputline);
                stringstream iss(inputline);
                for (int i=0;i<2*s_num-3;i++) {
                    std::getline(iss,colo,',');
                    stringstream convertor(colo);
                    convertor >> joint_raw[i];
                };
                joint_input[0] = 0.0;
                for(int i=0;i<s_num-2;i++){
                    joint_input[2*i+1]   = fabs(joint_raw[2*i]);
                    joint_input[2*i+2] =      joint_raw[2*i+1];
                };        
                frequency = fabs(joint_raw[s_num*2-5]);
                spring_stiff = fabs(joint_raw[s_num*2-4]);
        
                /// Applying Torque for actuators
                for (int i=0;i<s_num-2;i++) {
                    joint[i] = this->model->GetJoints()[i];
                    {
                        pos[i] = joint[i]->Position(0);
                        vel[i] = joint[i]->GetVelocity(0);
                        voltage[i] = joint_input[2*i+1]*sin(2*PI*(frequency*simu_time+joint_input[2*i]));
                        if(voltage[i]>15.0){
                            voltage[i]=15.0;
                        }
                        else if (voltage[i]<(-15.0)) {
                            voltage[i]=-15.0;
                        };
                        torque_actuator[i] = (voltage[i]-0.009079*vel[i])/90*0.009079*ac_ratio;
                        torque_spring[i] = -spring_stiff*pos[i]*ac_ratio;
                        joint[i]->SetForce(0, torque_spring[i]+torque_actuator[i]);
                    };
                };           // End of loop
                /// Applying Torque for passive joint
                joint[s_num-2] = this->model->GetJoints()[s_num-2];
                {
                    pos[s_num-2] = joint[s_num-2]->Position(0);
                    torque_spring[s_num-2] = (- stiff_ratio*spring_stiff*pos[s_num-2]) *ac_ratio;
                    joint[s_num-2]->SetForce(0, torque_spring[s_num-2]);
                };
            };
            ////////////////////////////////////////////////////////////////////////////////////////////////////////
        
             ///////////////////////////////////////////// Recording
             FILE *IndicateFile;
             IndicateFile=fopen("/home/donghao/result/DD/indicator.csv","r");
             fscanf(IndicateFile, "%d %d",&trial,&rollout);
             fclose(IndicateFile);
             if (trial>=1 && rollout==0){
                 fstream hydro_param_record;
                 if (simu_step == record_step){
                     hydro_param_record.open("/home/donghao/result/data/"+to_string(trial)+"/actuator_record.csv",ios::out);
                     hydro_param_record<< spring_stiff <<','<< stiff_ratio <<','<<ac_ratio<<','<< s_num<<endl;
                     hydro_param_record.close();
                 }
             }
            // /// Actuator parameter recording
             if (trial>=1 && simu_step == record_step  && rollout<3){
                 actuator_param.open("/home/donghao/result/data/"+to_string(trial)+"/"+to_string(rollout)+"_actuator_param.csv",ios::out);
                 for (int i=0;i<s_num*2-4;i++) {
                     actuator_param<<joint_input[i]<<',';
                 }
                 actuator_param<<frequency<<','<<spring_stiff<<','<<stiff_ratio<<endl;
                 actuator_param.close();
                 }
             /// Actuator parameter recording
             if (trial>=1 && simu_step >= record_step && rollout<3){
                 if(simu_step == record_step){
                      actuator_output.open("/home/donghao/result/data/"+to_string(trial)+"/"+to_string(rollout)+"_actuator_output.csv",ios::out);
                 }
                 else {
                      actuator_output.open("/home/donghao/result/data/"+to_string(trial)+"/"+to_string(rollout)+"_actuator_output.csv",ios::app);
        
                      actuator_output<<simu_time;
                      for(int i=0;i<s_num-1;i++){
                          actuator_output<<','<<torque_actuator[i]<<','<<torque_spring[i];
                      }
                      actuator_output<<endl;
                 }
                 actuator_output.close();
             }
        //////////////////////////////////////////////////////////////////////////////////////////////////////////
        };       // End of OnUpdateS
    
        private: physics::JointPtr joint[10];
    
        private: event::ConnectionPtr updateConnection;
    
        private: physics::ModelPtr model;
    
        private: common::Time prevUpdateTime;
    
    };
    GZ_REGISTER_MODEL_PLUGIN(ActuatorPlugin);
};
#endif

