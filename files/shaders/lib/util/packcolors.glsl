#ifndef LIB_UTIL_PACKCOLORS
#define LIB_UTIL_PACKCOLORS


    uniform bool isReflection;
    uniform bool isRefraction;

    float toFloat(float a, float b)
    {
        return floor(min(1.0, a) * 63.0) + min(b, 0.99);
    }

    void fromFloat(float encoded, inout float a, inout float b)
    {
        a = floor(encoded) / 63.0;
        b = encoded - floor(encoded);
    }

    vec4 encode(vec4 scene, vec4 normals)
    {
        if (isReflection || isRefraction)
            return scene;

        return -1.0 * vec4(toFloat(scene.r, normals.r), toFloat(scene.g, normals.g), toFloat(scene.b, normals.b), toFloat(scene.a, normals.a));
    }

    void decode(vec4 encoded, inout vec4 scene, inout vec4 normals)
    {
        if (encoded.r < 0.0 && encoded.g < 0.0 && encoded.b < 0.0)
        {
            encoded *= -1.0;
            fromFloat(encoded.r, scene.r, normals.r);
            fromFloat(encoded.g, scene.g, normals.g);
            fromFloat(encoded.b, scene.b, normals.b);
            fromFloat(encoded.a, scene.a, normals.a);
        }
        else
            scene = clamp(encoded, 0.0, 1.0);
    }

#endif
