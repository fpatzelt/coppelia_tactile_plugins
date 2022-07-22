// Copyright 2016 Coppelia Robotics AG. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// -------------------------------------------------------------------
// Authors:
// Federico Ferri <federico.ferri.it at gmail dot com>
// -------------------------------------------------------------------

#include "coppelia_ros_contact.h"
#include "simPlusPlus/Plugin.h"
#include "simPlusPlus/Handle.h"
#include "stubs.h"
#include "simLib.h"

using namespace Eigen;

// an example data structure to hold data across multiple calls
struct ExampleObject
{
    int a = 0;
    int b = 0;
    std::vector<int> seq;
};


struct TactileSensor
{
    std::string object_name;
    int object_handle;
    ros::Publisher contact_pub;
    ros::Publisher tactile_pub;
    std::string bumper_topic_name;
    std::string tactile_topic_name;
};

struct Collision
{
    int num_contacts;
    std::vector<Vector3d> forces;
    std::vector<Vector3d> contact_positions;
    std::vector<Vector3d> normals;
};

// conversion from C pointer to string handle and vice-versa:
template<> std::string sim::Handle<ExampleObject>::tag() { return "Example.Object"; }

class Plugin : public sim::Plugin
{
public:
    void onStart()
    {
        if(!registerScriptStuff())
            throw std::runtime_error("script stuff initialization failed");

        setExtVersion("0.0.0");
        setBuildDate(BUILD_DATE);
        sim::addLog(sim_verbosity_msgs,"start info");
    }

    void onScriptStateDestroyed(int scriptID)
    {
        for(auto obj : handles.find(scriptID))
            delete handles.remove(obj);
    }

    void createObject(createObject_in *in, createObject_out *out)
    {
        auto obj = new ExampleObject;

        out->handle = handles.add(obj, in->_.scriptID);
        sim::addLog(sim_verbosity_msgs,"create info");
    }

    void destroyObject(destroyObject_in *in, destroyObject_out *out)
    {
        auto obj = handles.get(in->handle);

        delete handles.remove(obj);
    }

    void setData(setData_in *in, setData_out *out)
    {
        auto obj = handles.get(in->handle);

        if(!obj->seq.empty())
            sim::addLog(sim_verbosity_warnings, "current sequence not empty");

        obj->a = in->a;
        obj->b = in->b;
    }

    void compute(compute_in *in, compute_out *out)
    {
        auto obj = handles.get(in->handle);

        obj->seq.push_back(obj->a + obj->b + 1);
        obj->a = obj->b;
        obj->b = obj->seq.back();
        out->currentSize = obj->seq.size();

    }

    void getOutput(getOutput_in *in, getOutput_out *out)
    {
        auto obj = handles.get(in->handle);

        out->output = obj->seq;
    }

    void handleTactileSensor(TactileSensor &ts) {
        std::map<int, Collision> collisions;
        int index = 0;
        while (true) {
            int objectHandles[2];
            float c[9];
            int r = simGetContactInfo(sim_handle_all, ts.object_handle, index + sim_handleflag_extended, objectHandles, c);            
            if (r > 0) {
                int objHandle1 = objectHandles[1];
                Vector3d contact_pos(c[0],c[1],c[2]);
                Vector3d force(c[3], c[4], c[5]);
                Vector3d normal(c[6], c[7], c[8]);
                if (collisions.count(objHandle1) > 0) {
                    collisions[objHandle1].num_contacts++;
                    collisions[objHandle1].forces.push_back(force);
                    collisions[objHandle1].contact_positions.push_back(contact_pos);
                    collisions[objHandle1].normals.push_back(normal);

                } else {
                    Collision col;
                    col.num_contacts = 1;
                    col.forces.push_back(force);
                    col.contact_positions.push_back(contact_pos);
                    col.normals.push_back(normal);
                    collisions[objHandle1] = col;
                }
                ++index;
            } else {
                break;
            } 
        }
        gazebo_msgs::ContactsState contact_state_msg;
        int n = collisions.size();
        if (n == 0) {
            return;
        }
        int i = 0;
        ros::Time time = ros::Time::now();
        Vector3d total_force0(0,0,0);
        Vector3d total_normal(0,0,0);
        Vector3d total_position(0,0,0);
        double total_force_lengths = 0;
        double total_normal_lengths;
        for (const auto& [objHandle1, col] : collisions) {
            Vector3d total_force(0,0,0);
            gazebo_msgs::ContactState state;
            state.collision1_name = ts.object_name;
            state.collision2_name = simGetObjectName(objHandle1);
            std::ostringstream stream;
            stream << "Debug:  i:(" << i << "/" << n << ")     my geom:" << state.collision1_name
                   << "   other geom:" << state.collision2_name
                   << "         time:" << time << std::endl;
            state.info = stream.str();

            ++i;

            for (int j = 0; j < col.num_contacts; ++j) {
                geometry_msgs::Wrench wrench;
                wrench.force.x = col.forces[j][0];
                wrench.force.y = col.forces[j][1];
                wrench.force.z = col.forces[j][2];
                wrench.torque.x = 0;
                wrench.torque.y = 0;
                wrench.torque.z = 0;
                state.wrenches.push_back(wrench);

                total_force += col.forces[j];


                geometry_msgs::Vector3 contact_normal;
                contact_normal.x = col.normals[j][0];
                contact_normal.y = col.normals[j][1];
                contact_normal.z = col.normals[j][2];
                state.contact_normals.push_back(contact_normal);

                total_normal += col.normals[j];

                total_normal_lengths += col.normals[j].norm();

                double force_length = col.forces[j].norm();
                
                geometry_msgs::Vector3 contact_position;
                contact_position.x = col.contact_positions[j][0];
                contact_position.y = col.contact_positions[j][1];
                contact_position.z = col.contact_positions[j][2];
                state.contact_positions.push_back(contact_position);

                total_position += force_length * col.contact_positions[j];
                total_force_lengths += force_length;

                state.depths.push_back(0);
            }
            geometry_msgs::Wrench total_wrench;
            total_wrench.force.x = total_force[0];
            total_wrench.force.y = total_force[1];
            total_wrench.force.z = total_force[2];
            total_wrench.torque.x = 0;
            total_wrench.torque.y = 0;
            total_wrench.torque.z = 0;
            state.total_wrench = total_wrench;
            contact_state_msg.states.push_back(state);
            total_force0 += total_force;
        }
        // normalize the normal
        if (total_normal_lengths != 0)
        {
            total_normal = total_normal / total_normal_lengths;
        }
        geometry_msgs::Vector3 tot_normal;
        tot_normal.x = total_normal[0];
        tot_normal.y = total_normal[1];
        tot_normal.z = total_normal[2];

        // compute average
        Vector3d average_position(0,0,0);
        if (total_force_lengths != 0) {
            average_position = total_position / total_force_lengths;
        }
        geometry_msgs::Point avg_position;
        avg_position.x = average_position[0];
        avg_position.y = average_position[1];
        avg_position.z = average_position[2];
    
        geometry_msgs::Wrench total_wrench;
        total_wrench.force.x = total_force0[0];
        total_wrench.force.y = total_force0[1];
        total_wrench.force.z = total_force0[2];
        total_wrench.torque.x = 0;
        total_wrench.torque.y = 0;
        total_wrench.torque.z = 0;

        // fill tactile message
        tactile_msgs::TactileContact tactile_contact_msg;
        tactile_contact_msg.name = ts.bumper_topic_name;
        tactile_contact_msg.position = avg_position;
        tactile_contact_msg.normal = tot_normal;
        tactile_contact_msg.wrench = total_wrench;
       
        // define frame name and stamp
        tactile_contact_msg.header.frame_id = "world";
        contact_state_msg.header.frame_id = "world";

        // adjust time
        tactile_contact_msg.header.stamp = time;
        contact_state_msg.header.stamp = time;

            
        ts.tactile_pub.publish(tactile_contact_msg);
        ts.contact_pub.publish(contact_state_msg);


    }



    void onModuleHandleInSensingPart(char *name)
    {

    for (auto ts : tactile_sensors) {
        handleTactileSensor(ts);     
    }
    }  
    void onSimulationAboutToStart()
    {

        simAddLog("ROSContact",sim_verbosity_msgs,"sim starting");
        if(!getenv("ROS_MASTER_URI"))
        {
            simAddLog("ROSContact",sim_verbosity_msgs,"ROS_MATER_URI is not set");
            throw std::runtime_error("ROS_MASTER_URI is not set");
            return;
        }

        if(!initialize())
        {
            simAddLog("ROSContact",sim_verbosity_msgs,"ROS master is not running");
            throw std::runtime_error("ROS master is not running");
        }
        if (!ros::isInitialized())
        {
            simAddLog("ROSContact",sim_verbosity_msgs,"ROS not initialized!");
            //ROS_FATAL_STREAM("No ROS core started A ROS node for Coppelia sim has not been initialized, unable to load plugin.");
            return;
        } 
        simAddLog("ROSContact",sim_verbosity_msgs,"ROS  initialized!");

        tactile_sensors.clear();

        const std::string urdf_string = getURDF(robot_description_);
        TiXmlDocument xml_doc;
        xml_doc.Parse(urdf_string.c_str());

        //urdf::Model urdf_model;
        //const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : NULL;
        //urdf::LinkConstSharedPtr root = urdf_model.getRoot();
        parseURDF(&xml_doc);
        
    }

    void registerTactileSensor(std::string obj_name, int obj_handle, std::string bumper_topic_name, std::string tactile_topic_name) {
        ros::Publisher tactile_pub = nh->advertise<tactile_msgs::TactileContact>(std::string(tactile_topic_name), 1);
        ros::Publisher contact_pub = nh->advertise<gazebo_msgs::ContactsState>(std::string(bumper_topic_name), 1);

        auto ts = new TactileSensor;
        ts->object_name = obj_name;
        ts->object_handle = obj_handle;
        ts->contact_pub = contact_pub;
        ts->tactile_pub = tactile_pub;
        ts->bumper_topic_name = bumper_topic_name;
        ts->tactile_topic_name = tactile_topic_name;
        tactile_sensors.push_back(*ts);
    }


    void parseURDF(TiXmlDocument *xml_doc)
    {
         
    TiXmlElement *robot_xml = xml_doc->FirstChildElement("robot");
    if (!robot_xml)
    {
      ROS_ERROR("Could not find the 'robot' element in the xml file");
      return;
    }

    ROS_DEBUG("Parsing robot xml");
    if (!robot_xml) return ;

    // Get robot name
    const char *name = robot_xml->Attribute("name");
    if (!name)
    {
      ROS_ERROR("No name given for the robot.");
      return;
    }

    simAddLog("ROSContact",sim_verbosity_msgs,"Find gazebo"); 
    // Get all Gazebo elements
    for (TiXmlElement* gazebo_xml = robot_xml->FirstChildElement("gazebo"); gazebo_xml; gazebo_xml = gazebo_xml->NextSiblingElement("gazebo"))
    {
        std::string reference = gazebo_xml->Attribute("reference");
        TiXmlElement *sensor_xml = gazebo_xml->FirstChildElement("sensor");
        if (sensor_xml)
        {
            simAddLog("ROSContact",sim_verbosity_msgs,"Found sensor"); 
            TiXmlElement *plugin_xml = sensor_xml->FirstChildElement("plugin");
            if (plugin_xml) // and (plugin_xml->Attribute("filename") == "libgazebo_ros_tactile.so"))
            {
                std::string plugin_fn = plugin_xml->Attribute("filename");
                if (plugin_fn == "libgazebo_ros_tactile.so") {
                    simAddLog("ROSContact",sim_verbosity_msgs,"Found plugin"); 
                    int objHandle=simGetObjectHandle(reference.c_str());
                    if (objHandle >= 0) {
                        TiXmlElement *bumper_topic_xml = plugin_xml->FirstChildElement("bumperTopicName");
                        std::string bumper_topic_name = "bumper_states";
                        if (bumper_topic_xml) 
                        {
                            simAddLog("ROSContact",sim_verbosity_msgs,"Bumper name found"); 
                            simAddLog("ROSContact",sim_verbosity_msgs,bumper_topic_xml->GetText()); 
                            bumper_topic_name = bumper_topic_xml->GetText();
                        } else
                        {
                            simAddLog("ROSContact",sim_verbosity_msgs,"Bumper name not found"); 
                        }
                        
                        TiXmlElement *tactile_topic_xml = plugin_xml->FirstChildElement("tactileTopicName");
                        std::string tactile_topic_name = "tactile_states";
                        if (tactile_topic_xml) 
                        {
                            simAddLog("ROSContact",sim_verbosity_msgs,"Tactile name found"); 
                            simAddLog("ROSContact",sim_verbosity_msgs,tactile_topic_xml->GetText()); 
                            tactile_topic_name = tactile_topic_xml->GetText();
                        } else
                        {
                            simAddLog("ROSContact",sim_verbosity_msgs,"Tactile name not found"); 
                        }
                        registerTactileSensor(reference, objHandle, bumper_topic_name, tactile_topic_name);
                    }
                }
            }
        }
    }
    }

      bool initialize()
    {
        int argc = 0;
#if _MSC_VER
        char **argv = nullptr;
#else
        char *argv[] = {};
#endif

        int node_name_length = 0;
        char *node_name = nullptr;
        node_name = simGetStringNamedParam("ROSInterface.nodeName", &node_name_length);

        ros::init(argc, argv, node_name && node_name_length ? node_name : "sim_ros_interface");

        if(node_name) simReleaseBuffer(node_name);

        if(!ros::master::check())
            return false;

        robot_namespace_ = ros::this_node::getNamespace();
        robot_description_ = "robot_description";
        nh = new ros::NodeHandle(robot_namespace_);


        return true;
    }

    std::string getURDF(std::string robot_description_)
    {
        std::string urdf_string;
        // search and wait for robot_description on param server
        while (urdf_string.empty())
        {   
            std::string search_param_name;
            if (nh->searchParam(robot_description_, search_param_name))
            {   
                ROS_INFO_ONCE_NAMED("gazebo_ros_control", "gazebo_ros_control plugin is waiting for model"
          " URDF in parameter [%s] on the ROS param server.", search_param_name.c_str());

                nh->getParam(search_param_name, urdf_string);
      }   
      else
      {   
            ROS_INFO_ONCE_NAMED("gazebo_ros_control", "gazebo_ros_control plugin is waiting for model"
          " URDF in parameter [%s] on the ROS param server.", robot_description_.c_str());

            nh->getParam(robot_description_, urdf_string);
      }   

      usleep(100000);
    }   
    ROS_DEBUG_STREAM_NAMED("gazebo_ros_control", "Recieved urdf from param server, parsing...");        
    return urdf_string;

    }


private:
      ros::NodeHandle *nh = NULL;
    sim::Handles<ExampleObject> handles;
    std::vector<TactileSensor> tactile_sensors;
    std::string robot_namespace_;
    std::string robot_description_;
};




SIM_PLUGIN(PLUGIN_NAME, PLUGIN_VERSION, Plugin)
#include "stubsPlusPlus.cpp"
