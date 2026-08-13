#version 330 core

@foreach clipPlaneIndex @clipPlaneList
    uniform mat4 clipPlane@clipPlaneIndex;
@endforeach

void applyClipPlanes(vec4 pos)
{
    @foreach clipPlaneIndex @clipPlaneList
        gl_ClipDistance[@clipPlaneIndex] = dot(clipPlane@clipPlaneIndex, pos);
    @endforeach
}