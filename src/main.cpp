#include "mujoco/mujoco.h"
#include "GLFW/glfw3.h"
#include "stdio.h"
#include "string.h"
#include <iostream>

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

// Mouse Interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// Mouse and Keyboard Control----------------------------------------------------------------------
// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}

// Mouse Scroll Callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


// Simulation Initialization----------------------------------------------------------------------
// Function to initialize MuJoCo simulation
void initMuJoCo() {
    // Load the model from XML file (replace "your_model.xml" with your MuJoCo model file)
    m = mj_loadXML("/app/models/unitree_a1/scene.xml", NULL, NULL, 0);

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
    window = glfwCreateWindow(1244, 700, "MuJoCo Visualization", NULL, NULL);

    if (!window) {
        std::cerr << "Error creating GLFW window." << std::endl;
        glfwTerminate();
        exit(1);
    }

    // Make the window's context current
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);
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

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    // camera view
    double arr_view[] = {90, -5, 5, 0.013, 0.0, 1.25};
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];
}

void renderMuJoCo(){
    // update scene and render
    viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    mjv_updateScene(m, d, &opt, &pert, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);
    glfwPollEvents();
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


// System Control ----------------------------------------------------------------------
void computeControlInput(const mjModel* m,  mjData* d) {

    float qpos_ref[] = {0, 5, 0.43, 1.0, 5, 0, 0, 5, 0, 0, 5, 0};
    float qvel_ref[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


    for(int i=0; i<12; i++){
        d->ctrl[i] = 0.1*(qpos_ref[i] - d->qpos[i]) + 0.02*(qvel_ref[i] - d->qvel[i]);
    }
}



int main(void)
{
    // Initialize MuJoCo
    initMuJoCo();

    // Initialize GLFW
    initGLFW();

    // Initialize Visualization
    initVis();
    
    // Controller
    mjcb_control = computeControlInput;

    // Run simulation until window closes
    while(!glfwWindowShouldClose(window)){
        // Simulation Step
        mjtNum simstart = d->time;
        while(d->time - simstart < 1.0/60.0){
            mj_step(m, d);
        }

        // Render the Model
        renderMuJoCo();     
    }

    // Cleanup Resources
    cleanup();

    return 0;
}
