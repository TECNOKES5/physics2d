#pragma once
#include "physics.h"
#include <GL/gl.h>
#include <cmath>

// ─── Renderer ─────────────────────────────────────────────────────────────────
// Uses legacy OpenGL immediate mode (no shader setup needed).
// Camera: world units mapped to [-aspect..aspect] x [-1..1]
struct Renderer {
    int   screenW=800, screenH=600;
    float zoom   = 5.f; // world units visible vertically (half-height)
    Vec2  camPos = {0,0};

    void resize(int w, int h) {
        screenW=w; screenH=h;
        glViewport(0,0,w,h);
    }

    void begin() {
        glClearColor(0.08f,0.08f,0.12f,1.f);
        glClear(GL_COLOR_BUFFER_BIT);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        float aspect = (float)screenW / screenH;
        glOrtho(-aspect*zoom, aspect*zoom, -zoom, zoom, -1,1);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glTranslatef(-camPos.x, -camPos.y, 0);
    }

    // Convert screen pixel → world coords
    Vec2 screenToWorld(float sx, float sy) const {
        float aspect = (float)screenW/screenH;
        float wx = ((sx / screenW) * 2.f - 1.f) * aspect * zoom + camPos.x;
        float wy = (1.f - (sy / screenH) * 2.f) * zoom + camPos.y;
        return {wx, wy};
    }

    void drawCircle(Vec2 pos, float r, float angle, float cr, float cg, float cb, bool fill=true) {
        const int segs = 32;
        glPushMatrix();
        glTranslatef(pos.x, pos.y, 0);

        if (fill) {
            glColor4f(cr*0.4f, cg*0.4f, cb*0.4f, 1.f);
            glBegin(GL_TRIANGLE_FAN);
            glVertex2f(0,0);
            for (int i=0;i<=segs;i++) {
                float a = (float)i/segs * 2.f*(float)M_PI;
                glVertex2f(std::cos(a)*r, std::sin(a)*r);
            }
            glEnd();
        }

        // Outline
        glColor4f(cr, cg, cb, 1.f);
        glLineWidth(1.5f);
        glBegin(GL_LINE_LOOP);
        for (int i=0;i<segs;i++) {
            float a = (float)i/segs * 2.f*(float)M_PI;
            glVertex2f(std::cos(a)*r, std::sin(a)*r);
        }
        glEnd();

        // Orientation line
        glBegin(GL_LINES);
        glVertex2f(0,0);
        glVertex2f(std::cos(angle)*r, std::sin(angle)*r);
        glEnd();

        glPopMatrix();
    }

    void drawBox(Vec2 pos, float hw, float hh, float angle, float cr, float cg, float cb, bool fill=true) {
        float c=std::cos(angle), s=std::sin(angle);
        auto toWorld = [&](float lx, float ly) -> Vec2 {
            return { pos.x + lx*c - ly*s, pos.y + lx*s + ly*c };
        };
        Vec2 v[4] = { toWorld(-hw,-hh), toWorld(hw,-hh),
                      toWorld(hw, hh),  toWorld(-hw, hh) };

        if (fill) {
            glColor4f(cr*0.35f, cg*0.35f, cb*0.35f, 1.f);
            glBegin(GL_TRIANGLE_FAN);
            for (auto& vv : v) glVertex2f(vv.x, vv.y);
            glEnd();
        }

        glColor4f(cr, cg, cb, 1.f);
        glLineWidth(1.5f);
        glBegin(GL_LINE_LOOP);
        for (auto& vv : v) glVertex2f(vv.x, vv.y);
        glEnd();
    }

    void drawBody(const Body& b) {
        if (b.shape.type == ShapeType::Circle)
            drawCircle(b.pos, b.shape.radius, b.angle, b.cr, b.cg, b.cb);
        else
            drawBox(b.pos, b.shape.hw, b.shape.hh, b.angle, b.cr, b.cg, b.cb);
    }

    void drawWorld(const World& w) {
        for (auto* b : w.bodies) drawBody(*b);
    }

    // Simple text-free grid (optional, helps spatial sense)
    void drawGrid(float spacing=1.f) {
        float aspect = (float)screenW/screenH;
        float x0 = camPos.x - aspect*zoom, x1 = camPos.x + aspect*zoom;
        float y0 = camPos.y - zoom,         y1 = camPos.y + zoom;
        glColor4f(0.18f,0.18f,0.22f,1.f);
        glLineWidth(1.f);
        glBegin(GL_LINES);
        for (float x=std::floor(x0/spacing)*spacing; x<=x1; x+=spacing) {
            glVertex2f(x,y0); glVertex2f(x,y1);
        }
        for (float y=std::floor(y0/spacing)*spacing; y<=y1; y+=spacing) {
            glVertex2f(x0,y); glVertex2f(x1,y);
        }
        glEnd();
    }
};
