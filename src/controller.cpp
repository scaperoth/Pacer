/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <Pacer/utilities.h>
#include <sys/time.h>
#include <dlfcn.h>
#include <errno.h>
#include <boost/foreach.hpp>
#include <stdlib.h>     /* getenv */

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <vector>

using namespace Pacer;
  
Controller::Controller(): Robot(){

}

Controller::~Controller(){
  close_plugins();
}


std::vector<void*> handles;
bool Controller::close_plugins(){
  // close the loaded plugin libraries
  for(size_t i = 0; i < handles.size(); ++i){
    dlclose(handles[i]);
  }
  handles.clear();
  _update_priority_map.clear();
  return true;
}

typedef void (*init_t)(const boost::shared_ptr<Controller>, const char*);

bool Controller::init_plugins(){
  OUT_LOG(logDEBUG) << ">> Controller::init_plugins()";

  bool RETURN_FLAG = true;
  close_plugins();
  std::vector<init_t> INIT;

  // call the initializers, if any
  std::vector<std::string> plugin_names = get_data<std::vector<std::string> >("plugin.id");
  std::vector<std::string> plugin_files = get_data<std::vector<std::string> >("plugin.file");
  std::vector<int> plugin_active = get_data<std::vector<int> >("plugin.active");

  // Load all the plugins
  for(unsigned i=0;i<plugin_names.size();i++){
    if(plugin_active[i] == 0) continue;
    std::string filename = plugin_files[i];

  if (!getenv("PACER_PLUGIN_PATH"))
    throw std::runtime_error("Environment variable PACER_PLUGIN_PATH not defined");

    std::string pPath(getenv("PACER_PLUGIN_PATH"));
    std::string lib_path = pPath+"/"+filename;
    OUT_LOG(logINFO) << "Loading Plugin: " << plugin_names[i];
    OUT_LOG(logINFO) << "\tLIB: " << filename.c_str();
    OUT_LOG(logINFO) << "\tPATH: " << pPath.c_str();
    // attempt to read the file
    void* HANDLE = dlopen(lib_path.c_str(), RTLD_LAZY);
    if (!HANDLE)
    {
      std::cerr << "driver: failed to read plugin from " << filename << std::endl;
      std::cerr << "  " << dlerror() << std::endl;
      RETURN_FLAG = false;
    }
 
    handles.push_back(HANDLE);
 
    // attempt to load the initializer
    dlerror();
    INIT.push_back((init_t) dlsym(HANDLE, "init"));
    const char* dlsym_error = dlerror();
    if (dlsym_error)
    {
      std::cerr << "driver warning: cannot load symbol 'init' from " << filename << std::endl;
      std::cerr << "        error follows: " << std::endl << dlsym_error << std::endl;
      INIT.pop_back();
      RETURN_FLAG = false;
    } else {
      // Init the plugin
      (*INIT.back())(this->ptr(),plugin_names[i].c_str());
    }
  }
    
  OUT_LOG(logDEBUG) << "<< Controller::init_plugins()";
  return RETURN_FLAG;
}

// ============================================================================
// =========================== Begin Robot Controller =========================
// ============================================================================

void Controller::control(double t){
    OUT_LOG(logDEBUG) << ">> Controller::control(.)";
  // Import Robot Data
  static double last_time = -0.001;
  const double dt = t - last_time;
  
  OUTLOG(t,"virtual_time",logERROR);
  OUTLOG(dt,"virtual_time_step",logERROR);
  
  update();
  lock_state();
#ifdef USE_PLUGINS
  update_plugins(t);
#endif
  unlock_state();
 
  {
    // Enforce limits
    // Note: homogeneous limits
    static std::vector<double> load_max = get_data<std::vector<double> >("init.joint.limits.u");

    Ravelin::VectorNd u = get_joint_generalized_value(Pacer::Controller::load_goal);
    OUTLOG(u,"U_NO_LIMIT",logERROR);

    for (int i=0;i<u.rows(); i++) {
      if(u[i] > load_max[0]) u[i] = load_max[0];
      else if(u[i] < -load_max[0]) u[i] = -load_max[0];
    }
    OUTLOG(u,"U_WITH_LIMIT",logERROR);
    set_joint_generalized_value(Pacer::Controller::load_goal,u);
  }
  
  reset_contact();
  last_time = t;
  OUT_LOG(logDEBUG) << "<< Controller::control(.)";
}
// ===========================  END CONTROLLER  ===============================
// ============================================================================
