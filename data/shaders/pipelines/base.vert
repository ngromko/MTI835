#version 450

#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

layout (location = 0) in vec3 inPos;
layout (location = 1) in vec3 inNormal;
layout (location = 2) in vec2 inUV;
layout (location = 3) in float inHeat;

layout (binding = 0) uniform UBO 
{
	mat4 projectionMatrix;
	mat4 viewMatrix;
} ubo;

layout (location = 0) out vec2 outUV;
layout (location = 1) out vec3 outNormal;
layout (location = 2) out vec3 worldPos;
layout (location = 3) out float outHeat;

void main() 
{
    outHeat = clamp(inHeat/75.0f,0.0f,1.0f);
	outUV = inUV;
    outUV.t = 1.0 - outUV.t;
    outNormal = inNormal.xyz;
    worldPos = inPos;
    gl_Position =  ubo.projectionMatrix * ubo.viewMatrix * vec4(inPos, 1.0);
}
