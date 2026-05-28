#pragma once
#include <vector>
#include <cmath>
#include <functional>

// ─── Vec2 ─────────────────────────────────────────────────────────────────────
struct Vec2 {
    float x = 0, y = 0;
    Vec2() = default;
    Vec2(float x, float y) : x(x), y(y) {}

    Vec2  operator+(const Vec2& o) const { return {x+o.x, y+o.y}; }
    Vec2  operator-(const Vec2& o) const { return {x-o.x, y-o.y}; }
    Vec2  operator*(float s)        const { return {x*s, y*s}; }
    Vec2  operator/(float s)        const { return {x/s, y/s}; }
    Vec2& operator+=(const Vec2& o)       { x+=o.x; y+=o.y; return *this; }
    Vec2& operator-=(const Vec2& o)       { x-=o.x; y-=o.y; return *this; }
    Vec2& operator*=(float s)             { x*=s;   y*=s;   return *this; }
    Vec2  operator-()                const { return {-x, -y}; }

    float dot(const Vec2& o)  const { return x*o.x + y*o.y; }
    float cross(const Vec2& o)const { return x*o.y - y*o.x; }
    float lenSq()             const { return x*x + y*y; }
    float len()               const { return std::sqrt(lenSq()); }
    Vec2  norm()              const { float l = len(); return l>1e-8f?Vec2{x/l,y/l}:Vec2{}; }
    Vec2  perp()              const { return {-y, x}; }
};
inline Vec2 operator*(float s, const Vec2& v) { return v*s; }

// Rotate a vec2 by angle (radians)
inline Vec2 rotate(Vec2 v, float a) {
    float c = std::cos(a), s = std::sin(a);
    return { v.x*c - v.y*s, v.x*s + v.y*c };
}

// ─── Shape types ──────────────────────────────────────────────────────────────
enum class ShapeType { Circle, Box };

struct Shape {
    ShapeType type;
    float radius = 0;   // circle
    float hw = 0, hh = 0; // box half-extents

    static Shape circle(float r)         { Shape s; s.type=ShapeType::Circle; s.radius=r; return s; }
    static Shape box(float w, float h)   { Shape s; s.type=ShapeType::Box; s.hw=w/2; s.hh=h/2; return s; }
};

// ─── Rigid Body ───────────────────────────────────────────────────────────────
struct Body {
    Shape    shape;
    Vec2     pos, vel, force;
    float    angle=0, omega=0, torque=0;
    float    mass, invMass;
    float    inertia, invInertia;
    float    restitution = 0.4f;
    float    friction    = 0.3f;
    bool     isStatic    = false;

    // colour (r,g,b) 0-1
    float cr=1, cg=1, cb=1;

    Body(Shape s, Vec2 p, float m=1.f) : shape(s), pos(p), mass(m) {
        if (m <= 0.f || std::isinf(m)) { isStatic=true; invMass=0; inertia=1; invInertia=0; }
        else {
            invMass = 1.f/m;
            if (s.type==ShapeType::Circle) {
                inertia = 0.5f * m * s.radius * s.radius;
            } else {
                inertia = m * (s.hw*s.hw + s.hh*s.hh) / 3.f;
            }
            invInertia = 1.f/inertia;
        }
    }

    void applyForce(Vec2 f) { force += f; }
    void applyImpulse(Vec2 imp, Vec2 r) {
        vel   += imp * invMass;
        omega += r.cross(imp) * invInertia;
    }
    Vec2 velocityAt(Vec2 r) const { return vel + Vec2{-omega*r.y, omega*r.x}; }
};

// ─── Manifold (collision info) ─────────────────────────────────────────────────
struct Manifold {
    Body* a = nullptr;
    Body* b = nullptr;
    Vec2  normal;
    float depth = 0;
    Vec2  contacts[2];
    int   numContacts = 0;
    bool  valid = false;
};

// ─── Collision detection ──────────────────────────────────────────────────────
inline Manifold circleVsCircle(Body& a, Body& b) {
    Manifold m;
    Vec2  d    = b.pos - a.pos;
    float dist = d.len();
    float rSum = a.shape.radius + b.shape.radius;
    if (dist >= rSum || dist < 1e-8f) return m;
    m.a = &a; m.b = &b; m.valid = true;
    m.normal = d / dist;
    m.depth  = rSum - dist;
    m.contacts[0] = a.pos + m.normal * a.shape.radius;
    m.numContacts = 1;
    return m;
}

// Get the 4 vertices of a box body in world space
inline void boxVertices(const Body& b, Vec2 out[4]) {
    float hw = b.shape.hw, hh = b.shape.hh;
    Vec2 corners[4] = {{-hw,-hh},{hw,-hh},{hw,hh},{-hw,hh}};
    for (int i=0;i<4;i++) out[i] = b.pos + rotate(corners[i], b.angle);
}

// Project box onto axis, return [min,max]
inline void projectBox(const Body& b, Vec2 axis, float& mn, float& mx) {
    Vec2 verts[4]; boxVertices(b, verts);
    mn = mx = axis.dot(verts[0]);
    for (int i=1;i<4;i++) {
        float p = axis.dot(verts[i]);
        if (p<mn) mn=p;
        if (p>mx) mx=p;
    }
}

inline Manifold boxVsBox(Body& a, Body& b) {
    Manifold m;
    // SAT on 4 axes (2 per box)
    auto axes = [&](const Body& bod, Vec2 out[2]) {
        float c=std::cos(bod.angle), s=std::sin(bod.angle);
        out[0] = Vec2{ c, s};
        out[1] = Vec2{-s, c};
    };
    Vec2 axA[2], axB[2];
    axes(a,axA); axes(b,axB);
    Vec2 allAxes[4] = {axA[0],axA[1],axB[0],axB[1]};

    float minDepth = 1e30f;
    Vec2  bestAxis;
    for (auto& ax : allAxes) {
        float mnA,mxA,mnB,mxB;
        projectBox(a,ax,mnA,mxA);
        projectBox(b,ax,mnB,mxB);
        float overlap = std::min(mxA,mxB) - std::max(mnA,mnB);
        if (overlap <= 0) return m; // separating axis found
        if (overlap < minDepth) { minDepth=overlap; bestAxis=ax; }
    }
    // Ensure normal points from a to b
    Vec2 d = b.pos - a.pos;
    if (d.dot(bestAxis) < 0) bestAxis = bestAxis * -1.f;

    m.a=&a; m.b=&b; m.valid=true;
    m.normal = bestAxis.norm();
    m.depth  = minDepth;

    // Simple contact point: midpoint of overlap on edge
    m.contacts[0] = (a.pos + b.pos) * 0.5f;
    m.numContacts = 1;
    return m;
}

inline Manifold circleVsBox(Body& circ, Body& box) {
    Manifold m;
    // Transform circle centre into box local space
    Vec2 local = rotate(circ.pos - box.pos, -box.angle);
    float hw=box.shape.hw, hh=box.shape.hh;
    Vec2 closest = { std::max(-hw,std::min(hw,local.x)),
                     std::max(-hh,std::min(hh,local.y)) };
    Vec2 diff = local - closest;
    float distSq = diff.lenSq();
    float r = circ.shape.radius;
    if (distSq >= r*r) return m;

    m.a=&circ; m.b=&box; m.valid=true;
    float dist = std::sqrt(distSq);
    Vec2 localNorm = dist < 1e-8f ? Vec2{1,0} : diff/dist;
    m.normal   = rotate(localNorm, box.angle);
    m.depth    = r - dist;
    m.contacts[0] = circ.pos - m.normal * r;
    m.numContacts  = 1;
    return m;
}

// ─── Impulse resolution ───────────────────────────────────────────────────────
inline void resolveManifold(Manifold& mf) {
    Body& a = *mf.a;
    Body& b = *mf.b;

    // Positional correction (anti-sinking)
    const float slop   = 0.01f;
    const float percent= 0.4f;
    Vec2 correction = mf.normal * (percent * std::max(mf.depth - slop, 0.f) /
                                   (a.invMass + b.invMass));
    if (!a.isStatic) a.pos -= correction * a.invMass;
    if (!b.isStatic) b.pos += correction * b.invMass;

    float e = std::min(a.restitution, b.restitution);

    for (int i=0;i<mf.numContacts;i++) {
        Vec2 ra = mf.contacts[i] - a.pos;
        Vec2 rb = mf.contacts[i] - b.pos;

        Vec2 relVel = b.velocityAt(rb) - a.velocityAt(ra);
        float velAlongNormal = relVel.dot(mf.normal);
        if (velAlongNormal > 0) continue; // separating

        float raCrossN = ra.cross(mf.normal);
        float rbCrossN = rb.cross(mf.normal);
        float invMassSum = a.invMass + b.invMass
                         + raCrossN*raCrossN*a.invInertia
                         + rbCrossN*rbCrossN*b.invInertia;

        float j = -(1.f + e) * velAlongNormal / (invMassSum * mf.numContacts);
        Vec2 impulse = mf.normal * j;
        a.applyImpulse(-impulse, ra);
        b.applyImpulse( impulse, rb);

        // Friction
        Vec2 tangent = (relVel - mf.normal * velAlongNormal).norm();
        float jt = -relVel.dot(tangent) / (invMassSum * mf.numContacts);
        float mu = std::sqrt(a.friction*a.friction + b.friction*b.friction);
        Vec2 frictionImpulse = std::abs(jt) < j*mu
            ? tangent * jt
            : tangent * (-j * mu);
        a.applyImpulse(-frictionImpulse, ra);
        b.applyImpulse( frictionImpulse, rb);
    }
}

// ─── World ────────────────────────────────────────────────────────────────────
struct World {
    std::vector<Body*> bodies;
    Vec2  gravity{0, -9.81f};
    float accumulator = 0;
    const float fixedDt = 1.f/120.f;
    const int   solverIter = 8;

    ~World() { for (auto* b : bodies) delete b; }

    Body* add(Shape s, Vec2 pos, float mass=1.f) {
        auto* b = new Body(s, pos, mass);
        bodies.push_back(b);
        return b;
    }

    void step(float dt) {
        accumulator += dt;
        while (accumulator >= fixedDt) {
            integrate(fixedDt);
            accumulatorCollide();
            accumulator -= fixedDt;
        }
    }

private:
    void integrate(float dt) {
        for (auto* b : bodies) {
            if (b->isStatic) continue;
            b->vel   += (gravity + b->force * b->invMass) * dt;
            b->omega += b->torque * b->invInertia * dt;
            b->pos   += b->vel * dt;
            b->angle += b->omega * dt;
            b->force  = {};
            b->torque = 0;
            // Damping
            b->vel   *= 0.999f;
            b->omega *= 0.995f;
        }
    }

    void accumulatorCollide() {
        std::vector<Manifold> manifolds;
        int n = bodies.size();
        for (int i=0;i<n;i++)
        for (int j=i+1;j<n;j++) {
            Body& a = *bodies[i];
            Body& b = *bodies[j];
            if (a.isStatic && b.isStatic) continue;

            Manifold mf;
            if (a.shape.type==ShapeType::Circle && b.shape.type==ShapeType::Circle)
                mf = circleVsCircle(a,b);
            else if (a.shape.type==ShapeType::Box && b.shape.type==ShapeType::Box)
                mf = boxVsBox(a,b);
            else if (a.shape.type==ShapeType::Circle && b.shape.type==ShapeType::Box)
                mf = circleVsBox(a,b);
            else {
                mf = circleVsBox(b,a);
                if (mf.valid) mf.normal = mf.normal * -1.f;
            }
            if (mf.valid) manifolds.push_back(mf);
        }
        for (int iter=0; iter<solverIter; iter++)
            for (auto& mf : manifolds) resolveManifold(mf);
    }
};
