#version 330 core

@foreach clipPlaneIndex @clipPlaneList
    uniform vec4 clipPlane@clipPlaneIndex;
@endforeach

uniform bool isReflection;
varying float clip;

void applyClipPlanes(vec4 pos)
{
    clip = 1.0;
    @foreach clipPlaneIndex @clipPlaneList
#ifdef GL_EXT_clip_cull_distance
        if (isReflection) gl_ClipDistance[@clipPlaneIndex] = dot(clipPlane@clipPlaneIndex, pos);
#else
        clip = dot(clipPlane@clipPlaneIndex, pos);
#endif

    @endforeach
}
