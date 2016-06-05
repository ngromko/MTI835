/*
* Vulkan Example - Animated gears using multiple uniform buffers
*
* See readme.md for details
*
* Copyright (C) 2015 by Sascha Willems - www.saschawillems.de
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

struct Vertex
{
	float pos[3];
	float normal[3];
    float uv[2];
	float color[3];
};

class VulkanCube : public VulkanObject
{
private:
    bool selectable;
	struct UBO
	{
		glm::mat4 projection;
        glm::mat4 model;
		glm::mat4 view;
	};

    // Reference to example for getting memory types
    VulkanExampleBase *exampleBase;

	struct
	{
		VkBuffer buf;
		VkDeviceMemory mem;
	} vertexBuffer;

    void prepareUniformBuffer(glm::vec3 startPos);
    void prepareVertices(glm::vec3 halfSize,glm::vec3 color);
    void prepareRigidBody(glm::vec3 size,glm::vec3 startPos,float mass);

    btRigidBody* rbody;
public:

	VkDescriptorSet descriptorSet;

	void draw(VkCommandBuffer cmdbuffer, VkPipelineLayout pipelineLayout);

	void setupDescriptorSet(VkDescriptorPool pool, VkDescriptorSetLayout descriptorSetLayout);

    VulkanCube(VkDevice device, VulkanExampleBase *example,glm::vec3 halfSize,glm::vec3 color, glm::vec3 startPos,float mass);
    ~VulkanCube();

    btRigidBody* getRigidBody();

};

