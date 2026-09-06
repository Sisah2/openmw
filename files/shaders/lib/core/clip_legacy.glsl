#version 120

void applyClipPlanes(vec4 pos)
{
    gl_ClipVertex = pos;
}