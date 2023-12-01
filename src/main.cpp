#include "mujoco/mujoco.h"
#include "GLFW/glfw3.h"
#include "stdio.h"
#include "string.h"
#include <iostream>

char error[1000];

// Mujoco simulation variables
mjModel* m;
mjData* d;
mjrRect viewport = {0, 0, 0, 0};

// GLFW window variables
GLFWwindow* window;

// Mujoco Visualization Objects
mjvCamera cam;                      // abstract camera
mjvPerturb pert;                    // perturbation object
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// RUNNING SIMULATION -------------------------------------------------
// ./run_dev.sh <- rebuilds the docker container
// this should take you to a new "root" directory 
// app/ buil# ls <- lists the packages available
// F1, enter, enter : Attach to runnng container
// In the new terminal cd build 
// type "make"
// run ./main
// docker ps - shows process' running, use name to kill container

// simulate ../model/humanoid/humanoid.xml

// FROM MUJOCO MODEL-----------------------------------------------------
//   <actuator>
//     <position class="abduction" name="FR_hip" joint="FR_hip_joint"/>
//     <position class="hip" name="FR_thigh" joint="FR_thigh_joint"/>
//     <position class="knee" name="FR_calf" joint="FR_calf_joint"/>
//     <position class="abduction" name="FL_hip" joint="FL_hip_joint"/>
//     <position class="hip" name="FL_thigh" joint="FL_thigh_joint"/>
//     <position class="knee" name="FL_calf" joint="FL_calf_joint"/>
//     <position class="abduction" name="RR_hip" joint="RR_hip_joint"/>
//     <position class="hip" name="RR_thigh" joint="RR_thigh_joint"/>
//     <position class="knee" name="RR_calf" joint="RR_calf_joint"/>
//     <position class="abduction" name="RL_hip" joint="RL_hip_joint"/>
//     <position class="hip" name="RL_thigh" joint="RL_thigh_joint"/>
//     <position class="knee" name="RL_calf" joint="RL_calf_joint"/>
//   </actuator>

// mujoco boot camp https://pab47.github.io/mujoco.html

// Function to initialize MuJoCo simulation
void initMuJoCo() {
    // Load the model from XML file (replace "your_model.xml" with your MuJoCo model file)
    m = mj_loadXML("/app/models/agility_cassie/scene.xml", NULL, NULL, 0);

    if (!m) {
        std::cerr << "Error loading MuJoCo model." << std::endl;
        exit(1);
    }

    // Set the step time
    m->opt.timestep = 0.01;

    // Initialize MuJoCo data structure
    d = mj_makeData(m);
}

// Function to initialize GLFW window
void initGLFW() {
    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Error initializing GLFW." << std::endl;
        exit(1);
    }

    // Create a windowed mode window and its OpenGL context
    window = glfwCreateWindow(800, 600, "MuJoCo Visualization", NULL, NULL);

    if (!window) {
        std::cerr << "Error creating GLFW window." << std::endl;
        glfwTerminate();
        exit(1);
    }

    // Make the window's context current
    glfwMakeContextCurrent(window);
}

void initVis() {
    // Initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultPerturb(&pert);
    mjv_defaultOption(&opt);
    mjr_defaultContext(&con);

    // mjv_makeScene(const mjModel* m, mjvScene* scn, int maxgeom);
    mjv_makeScene(m, &scn, 1000);                  // space for 1000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_100);     // model-specific context

    // get framebuffer viewport
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
}

void renderMuJoCo(){
    // update scene and render
    mjv_updateScene(m, d, &opt, &pert, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);
}

// Function to clean up resources
void cleanup() {
    // Free MuJoCo model and data
    mj_deleteData(d);
    mj_deleteModel(m);

    // Terminate GLFW
    glfwTerminate();
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
}

// void computeControlInput(const mjModel* m, const mjData* d, double& u) {
//     // Example: Apply PD control on a joint position
//     int jointID = mj_name2id(m, mjOBJ_JOINT, "your_joint_name");
//     double desiredPosition = 0.0; // Replace with your desired position

//     double positionError = desiredPosition - d->qpos[jointID];
//     double velocityError = 0.0 - d->qvel[jointID];

//     u = 1.0 * positionError + 0.1 * velocityError;
// }

int main(void)
{
    // Initialize MuJoCo
    initMuJoCo();

    // Initialize GLFW
    initGLFW();

    // Initialize Visualization
    initVis();

    // run simulation for 10 seconds
    while(!glfwWindowShouldClose(window)){
        // Simulation Step
        mj_step(m, d);

        // Feedback control
        // double controlInput;
        // computeControlInput(m, d, controlInput);

        // Apply control input to the system (replace with your system-specific control logic)
        // For example, set joint torque in MuJoCo
        // int jointID = mj_name2id(m, mjOBJ_JOINT, "your_joint_name");
        // m->ctrl[jointID] = controlInput;

        // Render the Model
        renderMuJoCo();

        // Poll for and process events
        glfwPollEvents();        
    }

    // Cleanup Resources
    cleanup();

    return 0;
}
