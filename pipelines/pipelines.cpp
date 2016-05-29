/*
* Vulkan Example - Using different pipelines in one single renderpass
*
* Copyright (C) 2016 by Sascha Willems - www.saschawillems.de
*
* This code is licensed under the MIT license (MIT) (http://opensource.org/licenses/MIT)
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <vector>

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <gli/gli.hpp>

#include <vulkan/vulkan.h>

#include "vulkanexamplebase.h"

#include "btBulletDynamicsCommon.h"

#define VERTEX_BUFFER_BIND_ID 0
#define ENABLE_VALIDATION false

// Vertex layout for this example
struct Vertex {
	float pos[3];
	float col[3];
	float uv[2];
	float normal[3];
};

class VulkanExample: public VulkanExampleBase 
{
private:
	vkTools::VulkanTexture textureColorMap;
public:
	struct {
		int count;
		VkPipelineVertexInputStateCreateInfo inputState;
		std::vector<VkVertexInputBindingDescription> bindingDescriptions;
		std::vector<VkVertexInputAttributeDescription> attributeDescriptions;
	} vertices;

	struct {
		vkMeshLoader::MeshBuffer cube;
	} meshes;

    btDiscreteDynamicsWorld* dynamicsWorld;
    btRigidBody* rBody;

    struct {
        VkBuffer buffer;
        VkDeviceMemory memory;
        // Store the mapped address of the particle data for reuse
        void *mappedMemory;
        std::vector<Vertex> vertexs;
    } rigidBody;

	vkTools::UniformData uniformDataVS;

	// Same uniform buffer layout as shader
	struct {
		glm::mat4 projectionMatrix;
		glm::mat4 modelMatrix;
		glm::mat4 viewMatrix;
	} uboVS;

	VkPipelineLayout pipelineLayout;
	VkDescriptorSet descriptorSet;
	VkDescriptorSetLayout descriptorSetLayout;

	struct {
		VkPipeline solidColor;
		VkPipeline wireFrame;
		VkPipeline texture;
	} pipelines;

	VulkanExample() : VulkanExampleBase(ENABLE_VALIDATION)
	{
		zoom = -5.0f;
        rotation = glm::vec3(-35.0f, 25.0f, 0.0f);
		title = "Vulkan Example - Using pipelines";
	}

	~VulkanExample()
	{
		// Clean up used Vulkan resources 
		// Note : Inherited destructor cleans up resources stored in base class
		vkDestroyPipeline(device, pipelines.solidColor, nullptr);
		vkDestroyPipeline(device, pipelines.wireFrame, nullptr);
		vkDestroyPipeline(device, pipelines.texture, nullptr);
		
		vkDestroyPipelineLayout(device, pipelineLayout, nullptr);
		vkDestroyDescriptorSetLayout(device, descriptorSetLayout, nullptr);

		vkMeshLoader::freeMeshBufferResources(device, &meshes.cube);

		vkDestroyBuffer(device, uniformDataVS.buffer, nullptr);
		vkFreeMemory(device, uniformDataVS.memory, nullptr);

		textureLoader->destroyTexture(textureColorMap);
	}

	void loadTextures()
	{
        textureLoader->loadTexture(
			getAssetPath() + "textures/crate_bc3.ktx", 
			VK_FORMAT_BC3_UNORM_BLOCK, 
			&textureColorMap);
	}

	void buildCommandBuffers()
	{		 
		VkCommandBufferBeginInfo cmdBufInfo = vkTools::initializers::commandBufferBeginInfo();

		VkClearValue clearValues[2];
		clearValues[0].color = defaultClearColor;
		clearValues[1].depthStencil = { 1.0f, 0 };

		VkRenderPassBeginInfo renderPassBeginInfo = vkTools::initializers::renderPassBeginInfo();
		renderPassBeginInfo.renderPass = renderPass;
		renderPassBeginInfo.renderArea.offset.x = 0;
		renderPassBeginInfo.renderArea.offset.y = 0;
		renderPassBeginInfo.renderArea.extent.width = width;
		renderPassBeginInfo.renderArea.extent.height = height;
		renderPassBeginInfo.clearValueCount = 2;
		renderPassBeginInfo.pClearValues = clearValues;

		VkResult err;

		for (int32_t i = 0; i < drawCmdBuffers.size(); ++i)
		{
			// Set target frame buffer
			renderPassBeginInfo.framebuffer = frameBuffers[i];

			err = vkBeginCommandBuffer(drawCmdBuffers[i], &cmdBufInfo);
			assert(!err);

			vkCmdBeginRenderPass(drawCmdBuffers[i], &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

            VkViewport viewport = vkTools::initializers::viewport(
                (float)600.0f,
				(float)height,
				0.0f,
				1.0f);
            //vkCmdSetViewport(drawCmdBuffers[i], 0, 1, &viewport);

			VkRect2D scissor = vkTools::initializers::rect2D(
				width,
				height,
				0,
				0);
            //vkCmdSetScissor(drawCmdBuffers[i], 0, 1, &scissor);

			vkCmdBindDescriptorSets(drawCmdBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSet, 0, NULL);

			VkDeviceSize offsets[1] = { 0 };
            vkCmdBindVertexBuffers(drawCmdBuffers[i], VERTEX_BUFFER_BIND_ID, 1, &rigidBody.buffer, offsets);

			vkCmdSetLineWidth(drawCmdBuffers[i], 2.0f);

			// Left : Solid colored 
            viewport.x = (float)width / 3.0;
            viewport.width = (float)width / 3.0;
			vkCmdSetViewport(drawCmdBuffers[i], 0, 1, &viewport);
            vkCmdBindPipeline(drawCmdBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, pipelines.solidColor);
			
			vkCmdDraw(drawCmdBuffers[i], vertices.count, 1, 0, 0);

			// Center : Textured
			viewport.x = (float)width / 3.0;
            vkCmdBindVertexBuffers(drawCmdBuffers[i], VERTEX_BUFFER_BIND_ID, 1, &meshes.cube.vertices.buf, offsets);
			vkCmdSetViewport(drawCmdBuffers[i], 0, 1, &viewport);
			vkCmdBindPipeline(drawCmdBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, pipelines.texture);
			vkCmdSetLineWidth(drawCmdBuffers[i], 2.0f);
			vkCmdDraw(drawCmdBuffers[i], vertices.count, 1, 0, 0);

			// Right : Wireframe 
			viewport.x = (float)width / 3.0 + (float)width / 3.0;
			vkCmdSetViewport(drawCmdBuffers[i], 0, 1, &viewport);
			vkCmdBindPipeline(drawCmdBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, pipelines.wireFrame);
			vkCmdDraw(drawCmdBuffers[i], vertices.count, 1, 0, 0);

			vkCmdEndRenderPass(drawCmdBuffers[i]);

			err = vkEndCommandBuffer(drawCmdBuffers[i]);
			assert(!err);
		}
	}
    
    void buildBulletScene(){
        ///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
        btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();

        ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
        btCollisionDispatcher* dispatcher = new	btCollisionDispatcher(collisionConfiguration);

        ///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
        btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();

        ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
        btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

       dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,overlappingPairCache,solver,collisionConfiguration);

        dynamicsWorld->setGravity(btVector3(0,-10,0));

        btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(1.),btScalar(1.),btScalar(1.)));

        btVector3 localInertia(0,0,0);

        btTransform groundTransform;
        groundTransform.setIdentity();

        btQuaternion qut = btQuaternion(btVector3(1,0,0),glm::radians(rotation.x));

        qut += btQuaternion(btVector3(0,1,0),glm::radians(rotation.y));

        qut += btQuaternion(btVector3(0,0,1),glm::radians(rotation.z));
        //groundTransform.setRotation(qut);

        btMatrix3x3 bob = btMatrix3x3(uboVS.modelMatrix[0].x,uboVS.modelMatrix[1].x,uboVS.modelMatrix[2].x,
                uboVS.modelMatrix[0].y,uboVS.modelMatrix[1].y,uboVS.modelMatrix[2].y,
                uboVS.modelMatrix[0].z,uboVS.modelMatrix[1].z,uboVS.modelMatrix[2].z);

groundTransform.setBasis(bob);

        btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);

        btRigidBody::btRigidBodyConstructionInfo rbInfo(0.,myMotionState,groundShape,localInertia);
        rBody = new btRigidBody(rbInfo);
        //add the body to the dynamics world
        dynamicsWorld->addRigidBody(rBody);

        btVector3 bobo = rBody->getWorldTransform().getOrigin();
        std::cout << " box ori " << bobo.x() << " " << bobo.y() << " " << bobo.z() << std::endl;

        bob = rBody->getWorldTransform().getBasis();

        bobo = bob*btVector3(1,1,1);

        std::cout << " trans ori1 " << bobo.x() << " " << bobo.y() << " " << bobo.z() << std::endl;

        glm::vec4 tob = glm::vec4(1,1,1,1);

        tob = uboVS.modelMatrix*tob;

        std::cout << " trans ori2 " << tob.x << " " << tob.y << " " << tob.z << std::endl;


    }

	void draw()
	{
		VkResult err;

		// Get next image in the swap chain (back/front buffer)
		err = swapChain.acquireNextImage(semaphores.presentComplete, &currentBuffer);
		assert(!err);

		submitPostPresentBarrier(swapChain.buffers[currentBuffer].image);

		// Command buffer to be sumitted to the queue
		submitInfo.commandBufferCount = 1;
		submitInfo.pCommandBuffers = &drawCmdBuffers[currentBuffer];

		// Submit to queue
		err = vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
		assert(!err);

		submitPrePresentBarrier(swapChain.buffers[currentBuffer].image);

		err = swapChain.queuePresent(queue, currentBuffer, semaphores.renderComplete);
		assert(!err);

		err = vkQueueWaitIdle(queue);
		assert(!err);
	}

	// Create vertices and buffers for uv mapped cube
	void generateCube()
	{

		// Setup vertices
#define colred { 1.0f, 0.0f, 0.0f }
#define colgreen { 0.0f, 1.0f, 0.0f }
#define colblue { 0.0f, 0.0f, 1.0f }
#define d 1.0f

		std::vector<Vertex> vertexBuffer = {

			// -Y
			{ { d,-d, d }, colred,{ 1.0, 1.0 }, { 0.0f, 1.0f, 0.0f } },
			{ { -d,-d,-d }, colred,{ 0.0, 0.0 },{ 0.0f, 1.0f, 0.0f } },
			{ { d,-d,-d }, colred,{ 1.0, 0.0 },{ 0.0f, 1.0f, 0.0f } },
			{ { d,-d, d }, colred,{ 1.0, 1.0 },{ 0.0f, 1.0f, 0.0f } },
			{ { -d,-d, d }, colred,{ 0.0, 1.0 },{ 0.0f, 1.0f, 0.0f } },
			{ { -d,-d,-d }, colred,{ 0.0, 0.0 },{ 0.0f, 1.0f, 0.0f } },
			// +Y
			{ { d, d, d }, colred,{ 1.0, 1.0 },{ 0.0f, -1.0f, 0.0f } },
			{ { d, d,-d }, colred,{ 1.0, 0.0 },{ 0.0f, -1.0f, 0.0f } },
			{ { -d, d,-d }, colred,{ 0.0, 0.0 },{ 0.0f, -1.0f, 0.0f } },
			{ { d, d, d }, colred,{ 1.0, 1.0 },{ 0.0f, -1.0f, 0.0f } },
			{ { -d, d,-d }, colred,{ 0.0, 0.0 },{ 0.0f, -1.0f, 0.0f } },
			{ { -d, d, d }, colred,{ 0.0, 1.0 },{ 0.0f, -1.0f, 0.0f } },
			// -X
			{ { -d,-d,-d }, colblue,{ 0.0, 0.0 },{ -1.0f, 0.0f, 0.0f } },
			{ { -d,-d, d }, colblue,{ 0.0, 1.0 },{ -1.0f, 0.0f, 0.0f } },
			{ { -d, d, d }, colblue,{ 1.0, 1.0 },{ -1.0f, 0.0f, 0.0f } },
			{ { -d,-d,-d }, colblue,{ 0.0, 0.0 },{ -1.0f, 0.0f, 0.0f } },
			{ { -d, d, d }, colblue,{ 1.0, 1.0 },{ -1.0f, 0.0f, 0.0f } },
			{ { -d, d,-d }, colblue,{ 1.0, 0.0 },{ -1.0f, 0.0f, 0.0f } },
			// +X
			{ { d, d, d }, colblue,{ 1.0, 1.0 },{ 1.0f, 0.0f, 0.0f } },
			{ { d,-d,-d }, colblue,{ 0.0, 0.0 },{ 1.0f, 0.0f, 0.0f } },
			{ { d, d,-d }, colblue,{ 1.0, 0.0 },{ 1.0f, 0.0f, 0.0f } },
			{ { d,-d,-d }, colblue,{ 0.0, 0.0 },{ 1.0f, 0.0f, 0.0f } },
			{ { d, d, d }, colblue,{ 1.0, 1.0 },{ 1.0f, 0.0f, 0.0f } },
			{ { d,-d, d }, colblue,{ 0.0, 1.0 },{ 1.0f, 0.0f, 0.0f } },
			// -Z
			{ { d, d,-d }, colgreen,{ 1.0, 1.0 },{ 0.0f, 0.0f, -1.0f } },
			{ { -d,-d,-d }, colgreen,{ 0.0, 0.0 },{ 0.0f, 0.0f, -1.0f } },
			{ { -d, d,-d }, colgreen,{ 0.0, 1.0 },{ 0.0f, 0.0f, -1.0f } },
			{ { d, d,-d }, colgreen,{ 1.0, 1.0 },{ 0.0f, 0.0f, -1.0f } },
			{ { d,-d,-d }, colgreen,{ 1.0, 0.0 },{ 0.0f, 0.0f, -1.0f } },
			{ { -d,-d,-d }, colgreen,{ 0.0, 0.0 },{ 0.0f, 0.0f, -1.0f } },
			// +Z
			{ { -d, d, d }, colgreen,{ 0.0, 1.0 },{ 0.0f, 0.0f, 1.0f } },
			{ { -d,-d, d }, colgreen,{ 0.0, 0.0 },{ 0.0f, 0.0f, 1.0f } },
			{ { d,-d, d }, colgreen,{ 1.0, 0.0 },{ 0.0f, 0.0f, 1.0f } },
			{ { d, d, d }, colgreen,{ 1.0, 1.0 },{ 0.0f, 0.0f, 1.0f } },
			{ { -d, d, d }, colgreen,{ 0.0, 1.0 },{ 0.0f, 0.0f, 1.0f } },
			{ { d,-d, d }, colgreen,{ 1.0, 0.0 },{ 0.0f, 0.0f, 1.0f } }

		};
#undef d

		vertices.count = vertexBuffer.size();

		createBuffer(
			VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
			vertexBuffer.size() * sizeof(Vertex),
			vertexBuffer.data(),
			&meshes.cube.vertices.buf,
            &meshes.cube.vertices.mem);

        for(int i=0;i<vertexBuffer.size();i++){
            vertexBuffer.at(i).pos[0]/=20;
            vertexBuffer.at(i).pos[1]/=20;
            vertexBuffer.at(i).pos[2]/=20;
        }
        createBuffer(
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
            vertexBuffer.size() * sizeof(Vertex),
            vertexBuffer.data(),
            &rigidBody.buffer,
            &rigidBody.memory);

        rigidBody.vertexs = vertexBuffer;

        VkResult err = vkMapMemory(device, rigidBody.memory, 0, vertexBuffer.size() * sizeof(Vertex), 0, &rigidBody.mappedMemory);
        assert(!err);
	}

	void prepareVertices()
	{
		// Binding description
		vertices.bindingDescriptions.resize(1);
		vertices.bindingDescriptions[0] =
			vkTools::initializers::vertexInputBindingDescription(
				VERTEX_BUFFER_BIND_ID,
				sizeof(Vertex),
				VK_VERTEX_INPUT_RATE_VERTEX);

		// Attribute descriptions
		// Describes memory layout and shader positions
		vertices.attributeDescriptions.resize(4);
		// Location 0 : Position
		vertices.attributeDescriptions[0] =
			vkTools::initializers::vertexInputAttributeDescription(
				VERTEX_BUFFER_BIND_ID,
				0,
				VK_FORMAT_R32G32B32_SFLOAT,
				0);
		// Location 1 : Color
		vertices.attributeDescriptions[1] =
			vkTools::initializers::vertexInputAttributeDescription(
				VERTEX_BUFFER_BIND_ID,
				1,
				VK_FORMAT_R32G32B32_SFLOAT,
				sizeof(float) * 3);
		// Location 3 : Texture coordinates
		vertices.attributeDescriptions[2] =
			vkTools::initializers::vertexInputAttributeDescription(
				VERTEX_BUFFER_BIND_ID,
				2,
				VK_FORMAT_R32G32_SFLOAT,
				sizeof(float) * 6);
		// Location 2 : Normal
		vertices.attributeDescriptions[3] =
			vkTools::initializers::vertexInputAttributeDescription(
				VERTEX_BUFFER_BIND_ID,
				3,
				VK_FORMAT_R32G32B32_SFLOAT,
				sizeof(float) * 8);

		vertices.inputState = vkTools::initializers::pipelineVertexInputStateCreateInfo();
		vertices.inputState.vertexBindingDescriptionCount = vertices.bindingDescriptions.size();
		vertices.inputState.pVertexBindingDescriptions = vertices.bindingDescriptions.data();
		vertices.inputState.vertexAttributeDescriptionCount = vertices.attributeDescriptions.size();
		vertices.inputState.pVertexAttributeDescriptions = vertices.attributeDescriptions.data();
	}

	void setupDescriptorPool()
	{
		// Example uses one ubo and one combined image sampler 
		std::vector<VkDescriptorPoolSize> poolSizes =
		{
			vkTools::initializers::descriptorPoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1),
			vkTools::initializers::descriptorPoolSize(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1),
		};

		VkDescriptorPoolCreateInfo descriptorPoolInfo =
			vkTools::initializers::descriptorPoolCreateInfo(
				poolSizes.size(),
				poolSizes.data(),
				2);

		VkResult vkRes = vkCreateDescriptorPool(device, &descriptorPoolInfo, nullptr, &descriptorPool);
		assert(!vkRes);
	}

	void setupDescriptorSetLayout()
	{
		std::vector<VkDescriptorSetLayoutBinding> setLayoutBindings =
		{
			// Binding 0 : Vertex shader uniform buffer
			vkTools::initializers::descriptorSetLayoutBinding(
				VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
				VK_SHADER_STAGE_VERTEX_BIT,
				0),
			// Binding 1 : Fragment shader image sampler
			vkTools::initializers::descriptorSetLayoutBinding(
				VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
				VK_SHADER_STAGE_FRAGMENT_BIT,
				1),
		};

		VkDescriptorSetLayoutCreateInfo descriptorLayout =
			vkTools::initializers::descriptorSetLayoutCreateInfo(
				setLayoutBindings.data(),
				setLayoutBindings.size());

		VkResult err = vkCreateDescriptorSetLayout(device, &descriptorLayout, nullptr, &descriptorSetLayout);
		assert(!err);

		VkPipelineLayoutCreateInfo pPipelineLayoutCreateInfo =
			vkTools::initializers::pipelineLayoutCreateInfo(
				&descriptorSetLayout,
				1);

		err = vkCreatePipelineLayout(device, &pPipelineLayoutCreateInfo, nullptr, &pipelineLayout);
		assert(!err);
	}

	void setupDescriptorSet()
	{
		VkDescriptorSetAllocateInfo allocInfo =
			vkTools::initializers::descriptorSetAllocateInfo(
				descriptorPool,
				&descriptorSetLayout,
				1);

		VkResult vkRes = vkAllocateDescriptorSets(device, &allocInfo, &descriptorSet);
		assert(!vkRes);

		// Color map image descriptor
		VkDescriptorImageInfo texDescriptorColorMap =
			vkTools::initializers::descriptorImageInfo(
				textureColorMap.sampler,
				textureColorMap.view,
				VK_IMAGE_LAYOUT_GENERAL);

		std::vector<VkWriteDescriptorSet> writeDescriptorSets =
		{
			// Binding 0 : Vertex shader uniform buffer
			vkTools::initializers::writeDescriptorSet(
			descriptorSet,
				VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
				0,
				&uniformDataVS.descriptor),
			// Binding 1 : Fragment shader image sampler
			vkTools::initializers::writeDescriptorSet(
				descriptorSet,
				VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
				1,
				&texDescriptorColorMap)
		};

		vkUpdateDescriptorSets(device, writeDescriptorSets.size(), writeDescriptorSets.data(), 0, NULL);
	}

	void preparePipelines()
	{
		VkPipelineInputAssemblyStateCreateInfo inputAssemblyState =
			vkTools::initializers::pipelineInputAssemblyStateCreateInfo(
				VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST,
				0,
				VK_FALSE);

		VkPipelineRasterizationStateCreateInfo rasterizationState =
			vkTools::initializers::pipelineRasterizationStateCreateInfo(
				VK_POLYGON_MODE_FILL,
				VK_CULL_MODE_FRONT_BIT,
				VK_FRONT_FACE_CLOCKWISE,
				0);

		VkPipelineColorBlendAttachmentState blendAttachmentState =
			vkTools::initializers::pipelineColorBlendAttachmentState(
				0xf,
				VK_FALSE);

		VkPipelineColorBlendStateCreateInfo colorBlendState =
			vkTools::initializers::pipelineColorBlendStateCreateInfo(
				1,
				&blendAttachmentState);

		VkPipelineDepthStencilStateCreateInfo depthStencilState =
			vkTools::initializers::pipelineDepthStencilStateCreateInfo(
				VK_TRUE,
				VK_TRUE,
				VK_COMPARE_OP_LESS_OR_EQUAL);

		VkPipelineViewportStateCreateInfo viewportState =
			vkTools::initializers::pipelineViewportStateCreateInfo(1, 1, 0);

		VkPipelineMultisampleStateCreateInfo multisampleState =
			vkTools::initializers::pipelineMultisampleStateCreateInfo(
				VK_SAMPLE_COUNT_1_BIT,
				0);

		std::vector<VkDynamicState> dynamicStateEnables = {
			VK_DYNAMIC_STATE_VIEWPORT,
			VK_DYNAMIC_STATE_SCISSOR,
			VK_DYNAMIC_STATE_LINE_WIDTH
		};
		VkPipelineDynamicStateCreateInfo dynamicState =
			vkTools::initializers::pipelineDynamicStateCreateInfo(
				dynamicStateEnables.data(),
				dynamicStateEnables.size(),
				0);

		// Color pipeline
		// Load shaders
		std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages;

        shaderStages[0] = loadShader(getAssetPath() + "shaders/pipelines/basenoModel.vert.spv", VK_SHADER_STAGE_VERTEX_BIT);
		shaderStages[1] = loadShader(getAssetPath() + "shaders/pipelines/color.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);
	
		VkGraphicsPipelineCreateInfo pipelineCreateInfo =
			vkTools::initializers::pipelineCreateInfo(
				pipelineLayout,
				renderPass,
				0);

		pipelineCreateInfo.pVertexInputState = &vertices.inputState;
		pipelineCreateInfo.pInputAssemblyState = &inputAssemblyState;
		pipelineCreateInfo.pRasterizationState = &rasterizationState;
		pipelineCreateInfo.pColorBlendState = &colorBlendState;
		pipelineCreateInfo.pMultisampleState = &multisampleState;
		pipelineCreateInfo.pViewportState = &viewportState;
		pipelineCreateInfo.pDepthStencilState = &depthStencilState;
		pipelineCreateInfo.pDynamicState = &dynamicState;
		pipelineCreateInfo.stageCount = shaderStages.size();
		pipelineCreateInfo.pStages = shaderStages.data();

		// Textured pipeline
		VkResult err = vkCreateGraphicsPipelines(device, pipelineCache, 1, &pipelineCreateInfo, nullptr, &pipelines.solidColor);
        assert(!err);

        shaderStages[0] = loadShader(getAssetPath() + "shaders/pipelines/base.vert.spv", VK_SHADER_STAGE_VERTEX_BIT);
        shaderStages[1] = loadShader(getAssetPath() + "shaders/pipelines/color.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);
        pipelineCreateInfo.pStages = shaderStages.data();

		// Reuse most of the initial pipeline for the next pipelines and only change affected parameters
		// Cull back faces
		rasterizationState.cullMode = VK_CULL_MODE_BACK_BIT;

		// Pipeline for textured rendering
		// Use different fragment shader
		shaderStages[1] = loadShader(getAssetPath() + "shaders/pipelines/texture.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);
		err = vkCreateGraphicsPipelines(device, pipelineCache, 1, &pipelineCreateInfo, nullptr, &pipelines.texture);
		assert(!err);

		// Pipeline for wire frame rendering
		// Solid polygon fill
		rasterizationState.polygonMode = VK_POLYGON_MODE_LINE;
		// Use different fragment shader
		shaderStages[1] = loadShader(getAssetPath() + "shaders/pipelines/wireframe.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);
		err = vkCreateGraphicsPipelines(device, pipelineCache, 1, &pipelineCreateInfo, nullptr, &pipelines.wireFrame);
		assert(!err);
	}

	// Prepare and initialize uniform buffer containing shader uniforms
	void prepareUniformBuffers()
	{
		VkResult err;

		// Vertex shader uniform buffer block
		VkMemoryAllocateInfo allocInfo = vkTools::initializers::memoryAllocateInfo();
		VkMemoryRequirements memReqs;

		VkBufferCreateInfo bufferInfo = vkTools::initializers::bufferCreateInfo(
			VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
			sizeof(uboVS));

		err = vkCreateBuffer(device, &bufferInfo, nullptr, &uniformDataVS.buffer);
		assert(!err);
		vkGetBufferMemoryRequirements(device, uniformDataVS.buffer, &memReqs);
		allocInfo.allocationSize = memReqs.size;
		getMemoryType(memReqs.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT, &allocInfo.memoryTypeIndex);
		err = vkAllocateMemory(device, &allocInfo, nullptr, &uniformDataVS.memory);
		assert(!err);
		err = vkBindBufferMemory(device, uniformDataVS.buffer, uniformDataVS.memory, 0);
		assert(!err);

		uniformDataVS.descriptor.buffer = uniformDataVS.buffer;
		uniformDataVS.descriptor.offset = 0;
		uniformDataVS.descriptor.range = sizeof(uboVS);

		updateUniformBuffers();
	}

	void updateUniformBuffers()
	{
		uboVS.projectionMatrix = glm::perspective(glm::radians(60.0f), (float)(width / 3.0f) / (float)height, 0.1f, 256.0f);

		uboVS.viewMatrix = glm::translate(glm::mat4(), glm::vec3(0.0f, 0.0f, zoom));

		uboVS.modelMatrix = glm::mat4();
		uboVS.modelMatrix = glm::rotate(uboVS.modelMatrix, glm::radians(rotation.x), glm::vec3(1.0f, 0.0f, 0.0f));
		uboVS.modelMatrix = glm::rotate(uboVS.modelMatrix, glm::radians(rotation.y), glm::vec3(0.0f, 1.0f, 0.0f));
		uboVS.modelMatrix = glm::rotate(uboVS.modelMatrix, glm::radians(rotation.z), glm::vec3(0.0f, 0.0f, 1.0f));

		uint8_t *pData;
		VkResult err = vkMapMemory(device, uniformDataVS.memory, 0, sizeof(uboVS), 0, (void **)&pData);
		assert(!err);
		memcpy(pData, &uboVS, sizeof(uboVS));
		vkUnmapMemory(device, uniformDataVS.memory);
		assert(!err);
	}

	void prepare()
	{
		VulkanExampleBase::prepare();
		loadTextures();
		prepareVertices();
		prepareUniformBuffers();
		setupDescriptorSetLayout();
		generateCube();
		preparePipelines();
		setupDescriptorPool();
		setupDescriptorSet();
		buildCommandBuffers();
        buildBulletScene();
		prepared = true;
	}

	virtual void render()
	{
		if (!prepared)
			return;
		vkDeviceWaitIdle(device);
		draw();
		vkDeviceWaitIdle(device);
	}

	virtual void viewChanged()
	{
		updateUniformBuffers();
        btTransform groundTransform;
        groundTransform.setIdentity();

        btQuaternion qut = btQuaternion(btVector3(1,0,0),glm::radians(rotation.x));

        qut += btQuaternion(btVector3(0,1,0),glm::radians(rotation.y));

        qut += btQuaternion(btVector3(0,0,1),glm::radians(rotation.z));
        //groundTransform.setRotation(qut);

        btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);

        btMatrix3x3 bob = btMatrix3x3(uboVS.modelMatrix[0].x,uboVS.modelMatrix[1].x,uboVS.modelMatrix[2].x,
                uboVS.modelMatrix[0].y,uboVS.modelMatrix[1].y,uboVS.modelMatrix[2].y,
                uboVS.modelMatrix[0].z,uboVS.modelMatrix[1].z,uboVS.modelMatrix[2].z);

groundTransform.setBasis(bob);
        rBody->setWorldTransform(groundTransform);

        //rBody->getCollisionShape()->
	}

    virtual void pick(float x, float y){


        std::cout << "mouse " << x << " " << y <<  std::endl;
        float fw = (float)width;
        float fh = (float)height;
        float mx = (x-fw/3.0f)/(fw/3.0f);

        glm::vec4 lRayStart_NDC(
            (mx  - 0.5f) * 2.0f,
            (2.0f * y)/ fh -1.0f, // [0, 768] -> [-1,1]
            -1.0, // The near plane maps to Z=-1 in Normalized Device Coordinates
            1.0f
        );
        glm::vec4 lRayEnd_NDC(
            (mx  - 0.5f) * 2.0f,
            (2.0f * y)/ fh -1.0f,
            0.0,
            1.0f
        );

        glm::vec4 ori(
            0.0f,
            0.0f,
            5.0f,
            1.0f
        );

        std::cout << "orginiNDC " << lRayStart_NDC.x << " " << lRayStart_NDC.y <<" " << lRayStart_NDC.z << std::endl;


        // Faster way (just one inverse)
        glm::mat4 M = glm::inverse(uboVS.projectionMatrix * uboVS.viewMatrix);
        glm::vec4 lRayStart_world = M * lRayStart_NDC; lRayStart_world/=lRayStart_world.w;

         std::cout << "lRayEnd_NDC " << lRayStart_world.x << " " << lRayStart_world.y <<" " << lRayStart_world.z << std::endl;

        glm::vec4 lRayEnd_world   = M * lRayEnd_NDC  ; lRayEnd_world  /=lRayEnd_world.w;

        glm::vec3 lRayDir_world(lRayEnd_world - lRayStart_world);
        glm::vec3 out_origin(lRayStart_world);
        lRayDir_world = glm::normalize(lRayDir_world);

        glm::vec4 out_end = ori + (lRayStart_world-ori)*1000.0f;

        /*btCollisionWorld::ClosestRayResultCallback RayCallback(
            btVector3(out_origin.x, out_origin.y, out_origin.z),
            btVector3(out_end.x, out_end.y, out_end.z)
        );
        dynamicsWorld->rayTest(
            btVector3(out_origin.x, out_origin.y, out_origin.z),
            btVector3(out_end.x, out_end.y, out_end.z),
            RayCallback
        );*/
        btCollisionWorld::ClosestRayResultCallback RayCallback(
            btVector3(0.0f, 0.0f, 5.0f),
            btVector3(out_end.x, out_end.y, out_end.z)
        );
        dynamicsWorld->rayTest(
                    btVector3(0.0f, 0.0f, 5.0f),
                    btVector3(out_end.x, out_end.y, out_end.z),
            RayCallback
        );

        std::cout << "orgini " << out_origin.x << " " << out_origin.y <<" " << out_origin.z << std::endl;
       std::cout << "end " << out_end.x << " " << out_end.y <<" " << out_end.z << std::endl;

        if(RayCallback.hasHit()) {
             std::cout << "hit " << RayCallback.m_hitPointWorld.x() << " " << RayCallback.m_hitPointWorld.y() <<" " << RayCallback.m_hitPointWorld.z() << std::endl;

             for(int i=0;i<rigidBody.vertexs.size();i++){
                  rigidBody.vertexs.at(i).pos[0]+=RayCallback.m_hitPointWorld.x();
                  rigidBody.vertexs.at(i).pos[1]+=RayCallback.m_hitPointWorld.y();
                  rigidBody.vertexs.at(i).pos[2]+=RayCallback.m_hitPointWorld.z();
             }


             size_t size = rigidBody.vertexs.size() * sizeof(Vertex);
             memcpy(rigidBody.mappedMemory, rigidBody.vertexs.data(), size);

             for(int i=0;i<rigidBody.vertexs.size();i++){
                  rigidBody.vertexs.at(i).pos[0]-=RayCallback.m_hitPointWorld.x();
                  rigidBody.vertexs.at(i).pos[1]-=RayCallback.m_hitPointWorld.y();
                  rigidBody.vertexs.at(i).pos[2]-=RayCallback.m_hitPointWorld.z();
             }

        }else{
           std::cout << "background" << std::endl;
        }
    }

};

VulkanExample *vulkanExample;

#if defined(_WIN32)
LRESULT CALLBACK WndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	if (vulkanExample != NULL)
	{
		vulkanExample->handleMessages(hWnd, uMsg, wParam, lParam);
	}
	return (DefWindowProc(hWnd, uMsg, wParam, lParam));
}
#elif defined(__linux__) && !defined(__ANDROID__)
static void handleEvent(const xcb_generic_event_t *event)
{
	if (vulkanExample != NULL)
	{
		vulkanExample->handleEvent(event);
	}
}
#endif

// Main entry point
#if defined(_WIN32)
// Windows entry point
int APIENTRY WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR pCmdLine, int nCmdShow)
#elif defined(__ANDROID__)
// Android entry point
void android_main(android_app* state)
#elif defined(__linux__)
// Linux entry point
int main(const int argc, const char *argv[])
#endif
{
#if defined(__ANDROID__)
	// Removing this may cause the compiler to omit the main entry point 
	// which would make the application crash at start
	app_dummy();
#endif
	vulkanExample = new VulkanExample();
#if defined(_WIN32)
	vulkanExample->setupWindow(hInstance, WndProc);
#elif defined(__ANDROID__)
	// Attach vulkan example to global android application state
	state->userData = vulkanExample;
	state->onAppCmd = VulkanExample::handleAppCommand;
	state->onInputEvent = VulkanExample::handleAppInput;
	vulkanExample->androidApp = state;
#elif defined(__linux__)
	vulkanExample->setupWindow();
#endif
#if !defined(__ANDROID__)
	vulkanExample->initSwapchain();
	vulkanExample->prepare();
#endif
	vulkanExample->renderLoop();
	delete(vulkanExample);
#if !defined(__ANDROID__)
	return 0;
#endif
}
