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

class VulkanMesh : public VulkanObject
{
private:
    bool selectable;
	struct UBO
	{
		glm::mat4 projection;
        glm::mat4 model;
		glm::mat4 view;
	};
    vkMeshLoader::MeshBuffer* meshBuffer;
    void prepareRigidBody(std::vector<glm::vec3> bPoints, glm::mat4 startPos, float mass);

public:
    ~VulkanMesh();
    VulkanMesh(VkDevice mdevice, VulkanExampleBase *mexample, vkMeshLoader::MeshBuffer *eMeshBuffer, vkTools::VulkanTexture *eTexture, glm::mat4 atartModel, float mass, std::vector<glm::vec3> &bPoints);
    void draw(VkCommandBuffer cmdbuffer, VkPipelineLayout pipelineLayout);
};

