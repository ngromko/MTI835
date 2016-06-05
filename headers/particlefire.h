/*
* Vulkan Example - CPU based fire particle system 
*
* Copyright (C) 2016 by Sascha Willems - www.saschawillems.de
*
* This code is licensed under the MIT license (MIT) (http://opensource.org/licenses/MIT)
*/

#pragma once

#include <math.h>
#include <vector>

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_inverse.hpp>

#include "vulkanObject.h"
#include "vkObjectMotionState.h"

#include "vulkanexamplebase.h"
#include "btBulletDynamicsCommon.h"
#define PARTICLE_COUNT 512
#define PARTICLE_SIZE 10.0f

#define FLAME_RADIUS 0.1f

#define PARTICLE_TYPE_FLAME 0
#define PARTICLE_TYPE_SMOKE 1

struct Particle {
	glm::vec4 pos;
	glm::vec4 color;
	float alpha;
	float size;
	float rotation;
	uint32_t type;
	// Attributes not used in shader
	glm::vec4 vel;
	float rotationSpeed;
};

class VulkanFire : public VulkanObject
{
private:
    glm::vec3 emitterPos = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 minVel = glm::vec3(-3.0f, 0.5f, -3.0f);
    glm::vec3 maxVel = glm::vec3(3.0f, 7.0f, 3.0f);

	struct {
		VkBuffer buffer;
		VkDeviceMemory memory;
		// Store the mapped address of the particle data for reuse
		void *mappedMemory;
		// Size of the particle buffer in bytes
		size_t size;
    } particleVkBuffer;

	VkDescriptorSet descriptorSet;

    VulkanExampleBase *exampleBase;

	std::vector<Particle> particleBuffer;

    float rnd(float range);
    void initParticle(Particle *particle, glm::vec3 emitterPos);
    void transitionParticle(Particle *particle);
    void prepareParticles();
    // Prepare and initialize uniform buffer containing shader uniforms
    void prepareUniformBuffers();

public:
    void draw(VkCommandBuffer cmdbuffer, VkPipelineLayout pipelineLayout);
    void updateParticles(float);

    void loadTextures();
    void setupDescriptorSet(VkDescriptorPool pool, VkDescriptorSetLayout descriptorSetLayout,VkSampler,vkTools::VulkanTexture fire,vkTools::VulkanTexture smoke);
    VulkanFire(VkDevice device, VulkanExampleBase *example,glm::vec3 pos);

    ~VulkanFire();
};
