#include <GLFW/glfw3.h>
#include <cstdlib>
#include <cstdio>
#include <ctime>
#include "physics.h"
#include "renderer.h"

// ─── Globals ──────────────────────────────────────────────────────────────────
static World    g_world;
static Renderer g_renderer;
static bool     g_paused = false;
static int      g_spawnType = 0;   // 0=circle, 1=box

// Pretty palette
static float s_palette[][3] = {
    {0.96f,0.37f,0.37f}, // red
    {0.37f,0.86f,0.96f}, // cyan
    {0.55f,0.96f,0.37f}, // green
    {0.96f,0.85f,0.37f}, // yellow
    {0.75f,0.37f,0.96f}, // purple
    {0.96f,0.60f,0.37f}, // orange
    {0.37f,0.55f,0.96f}, // blue
};
static int s_palIdx = 0;

// ─── Setup scene ──────────────────────────────────────────────────────────────
void setupScene() {
    // Floor
    {
        auto* b = g_world.add(Shape::box(24.f, 0.5f), {0,-4.5f}, 0.f);
        b->restitution = 0.3f;
        b->cr=0.4f; b->cg=0.5f; b->cb=0.6f;
    }
    // Left wall
    {
        auto* b = g_world.add(Shape::box(0.5f, 12.f), {-9.5f,0}, 0.f);
        b->cr=0.4f; b->cg=0.5f; b->cb=0.6f;
    }
    // Right wall
    {
        auto* b = g_world.add(Shape::box(0.5f, 12.f), {9.5f,0}, 0.f);
        b->cr=0.4f; b->cg=0.5f; b->cb=0.6f;
    }
    // A ramp
    {
        auto* b = g_world.add(Shape::box(5.f, 0.3f), {-3.f,-1.5f}, 0.f);
        b->angle = 0.25f;
        b->cr=0.5f; b->cg=0.6f; b->cb=0.5f;
    }
    // Another ramp
    {
        auto* b = g_world.add(Shape::box(5.f, 0.3f), {3.5f,1.f}, 0.f);
        b->angle = -0.22f;
        b->cr=0.5f; b->cg=0.6f; b->cb=0.5f;
    }
}

// ─── Spawn body at world pos ───────────────────────────────────────────────────
void spawnAt(float wx, float wy) {
    float* col = s_palette[s_palIdx % 7];
    s_palIdx++;

    Body* b = nullptr;
    if (g_spawnType == 0) {
        float r = 0.2f + (rand()%10)*0.05f;
        b = g_world.add(Shape::circle(r), {wx,wy}, r*r*3.14f*2.f);
    } else {
        float w2 = 0.25f + (rand()%8)*0.07f;
        float h2 = 0.25f + (rand()%8)*0.07f;
        b = g_world.add(Shape::box(w2*2,h2*2), {wx,wy}, w2*h2*4.f*2.f);
        b->angle = ((rand()%100)/100.f - 0.5f) * 1.5f;
    }
    if (b) {
        b->cr = col[0]; b->cg = col[1]; b->cb = col[2];
        b->restitution = 0.3f + (rand()%5)*0.05f;
        b->vel = {((rand()%100)/50.f-1.f)*2.f, 0.f};
    }
}

// ─── GLFW Callbacks ────────────────────────────────────────────────────────────
void mouseButtonCB(GLFWwindow* win, int button, int action, int /*mods*/) {
    if (button==GLFW_MOUSE_BUTTON_LEFT && action==GLFW_PRESS) {
        double mx,my;
        glfwGetCursorPos(win,&mx,&my);
        Vec2 w = g_renderer.screenToWorld((float)mx,(float)my);
        spawnAt(w.x, w.y);
    }
}

void keyCB(GLFWwindow* win, int key, int /*sc*/, int action, int /*mods*/) {
    if (action!=GLFW_PRESS) return;
    if (key==GLFW_KEY_ESCAPE) glfwSetWindowShouldClose(win,1);
    if (key==GLFW_KEY_SPACE)  g_paused = !g_paused;
    if (key==GLFW_KEY_C)      g_spawnType = 0;
    if (key==GLFW_KEY_B)      g_spawnType = 1;
    if (key==GLFW_KEY_R) {
        // Reset world
        for (auto* b : g_world.bodies) delete b;
        g_world.bodies.clear();
        setupScene();
    }
    if (key==GLFW_KEY_G) g_world.gravity.y = -g_world.gravity.y;
}

void framebufferSizeCB(GLFWwindow* /*win*/, int w, int h) {
    g_renderer.resize(w,h);
}

// ─── Main ──────────────────────────────────────────────────────────────────────
int main() {
    srand((unsigned)time(nullptr));

    if (!glfwInit()) { fprintf(stderr,"glfwInit failed\n"); return 1; }
    glfwWindowHint(GLFW_SAMPLES, 4);

    GLFWwindow* win = glfwCreateWindow(800,600,"Physics2D  |  [C]ircle [B]ox [R]eset [G]ravity [Space]pause",nullptr,nullptr);
    if (!win) { fprintf(stderr,"glfwCreateWindow failed\n"); glfwTerminate(); return 1; }

    glfwMakeContextCurrent(win);
    glfwSwapInterval(1);

    glfwSetMouseButtonCallback(win, mouseButtonCB);
    glfwSetKeyCallback(win,         keyCB);
    glfwSetFramebufferSizeCallback(win, framebufferSizeCB);

    {
        int w,h; glfwGetFramebufferSize(win,&w,&h);
        g_renderer.resize(w,h);
    }

    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    setupScene();

    double prevTime = glfwGetTime();

    while (!glfwWindowShouldClose(win)) {
        double now = glfwGetTime();
        float  dt  = (float)(now - prevTime);
        prevTime   = now;
        if (dt > 0.05f) dt = 0.05f; // clamp spike frames

        if (!g_paused) g_world.step(dt);

        g_renderer.begin();
        g_renderer.drawGrid(1.f);
        g_renderer.drawWorld(g_world);

        glfwSwapBuffers(win);
        glfwPollEvents();
    }

    glfwDestroyWindow(win);
    glfwTerminate();
    return 0;
}
