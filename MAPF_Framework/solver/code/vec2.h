#if !defined(VEC2_H)
#define VEC2_H

struct Vec2
{
    f32 x;
    f32 y;
};
    
static Vec2
operator-(Vec2 v1, Vec2 v2)
{
    return {v1.x - v2.x, v1.y - v2.y};
}

static Vec2
operator+(Vec2 v1, Vec2 v2)
{
    return {v1.x + v2.x, v1.y + v2.y};
}

static Vec2
operator+(Vec2 v1, f32 x)
{
    return {v1.x + x, v1.y + x};
}

static void
operator+=(Vec2& v1, Vec2 v2)
{
    v1 = v1 + v2;
}

static Vec2
operator*(Vec2 v, f32 x)
{
    return {v.x * x, v.y * x};
}

static b32
operator==(Vec2 v1, Vec2 v2)
{
    return FloatEq(v1.x, v2.x) && FloatEq(v1.y, v2.y);
}

static f32
dot(Vec2 v1, Vec2 v2)
{
    return v1.x * v2.x + v1.y * v2.y;
}

static f32
distance(Vec2 v1, Vec2 v2)
{
    f32 x = v1.x - v2.x;
    f32 y = v1.y - v2.y;
    return sqrt(x * x + y * y);
}

static f32
length(Vec2 v)
{
    return sqrt(v.x * v.x + v.y * v.y);
}

// NOTE: Move from pos_1 at speed_1 to pos_2 at speed_2
static Vec2
compute_velocity(Vec2 pos_1, Vec2 pos_2, f32 speed_1, f32 speed_2)
{
    Vec2 result = {};

    f32 speed;
    if (FloatEq(speed_1, 0) && FloatEq(speed_2, 0))
    {
        speed = 0.5;
    }
    else
    {
        speed = (speed_1 + speed_2) / 2;
    }
    
    // old
    //f32 d = pos_2.x - pos_1.x;
    //f32 d_y = pos_2.y - pos_1.y;
    //f32 sign_x = d < 0 ? -1 : 1;
    //f32 sign_y = d_y < 0 ? -1 : 1;
    //result.x = sign_x * speed;
    //f32 angle = acos(d / length(pos_2 - pos_1));
    //result.y = sign_y * speed * tan(angle);

    // 02.04 after meeting Johan for PC
    f32 d = distance(pos_1, pos_2);
    f32 d_x = pos_2.x - pos_1.x;
    f32 d_y = pos_2.y - pos_1.y;

    f32 sign_x = d_x < 0 ? -1 : 1;
    f32 sign_y = d_y < 0 ? -1 : 1;

    f32 sin_alfa = abs(d_x) / d;
    f32 cos_alfa = abs(d_y) / d;

    result.x = sign_x * speed * sin_alfa;
    result.y = sign_y * speed * cos_alfa;

    //if (FloatEq(d_x, 0)) result.x = 0;
    //if (FloatEq(d_y, 0)) result.y = 0;

    return result;
}

#endif
