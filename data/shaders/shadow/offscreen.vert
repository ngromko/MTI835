#version 450

#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

layout (location = 0) in vec3 inPos;

layout (location = 0) out vec3 outPos;
layout (location = 1) out vec3 outLightPos;

layout (binding = 0) uniform UBO 
{
	mat4 projection;
	vec3 lightPos[2];
} ubo;

layout(push_constant) uniform PushConsts 
{
	mat4 view;
    uint lightIndex;
} pushConsts;
 
out gl_PerVertex 
{
	vec4 gl_Position;
};
 
void main()
{
	gl_Position = ubo.projection * pushConsts.view * vec4(inPos, 1.0);

	outPos = inPos;	
	outLightPos = ubo.lightPos[pushConsts.lightIndex].xyz; 
}