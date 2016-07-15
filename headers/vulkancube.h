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

    void prepareVertices(glm::vec3 halfSize, std::vector<BurningPoint> &bPoints, uint32_t objectNumber);
    void prepareRigidBody(glm::vec3 size,glm::vec4 startPos,float mass);

public:
    ~VulkanCube();
    VulkanCube(VkDevice mdevice, VulkanExampleBase *mexample, VkQueue queue, glm::vec3 halfSize, glm::mat4 atartModel, bool burn, float mass, uint32_t objectNumber, std::vector<BurningPoint> &points);
};

