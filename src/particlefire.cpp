#include "particlefire.h"

VulkanFire::VulkanFire(VkDevice device, VulkanExampleBase *example)
{
    this->device = device;
    this->exampleBase=example; 
}

VulkanFire::~VulkanFire()
{
    // Clean up used Vulkan resources
    // Note : Inherited destructor cleans up resources stored in base class

    vkUnmapMemory(device, computeUniformBuffer.memory);
    vkDestroyBuffer(device, computeUniformBuffer.buffer, nullptr);
    vkFreeMemory(device, computeUniformBuffer.memory, nullptr);

    vkDestroyPipeline(device,drawParticles,nullptr);
    vkDestroyPipeline(device,clickFire,nullptr);
    vkDestroyPipeline(device,updateParticles,nullptr);
    vkDestroyPipeline(device,propageFire,nullptr);
}

void VulkanFire::compute(VkCommandBuffer cmdbuffer)
{
    int bGroups  = 1;
    while(bGroups*512<computeUbo.bPointsCount){
        bGroups++;
    }
    int pGroups  = 1;
    while(pGroups*512<computeUbo.particleCount){
        pGroups++;
    }

    // Add memory barrier to ensure that the (rendering) vertex shader operations have finished
    // Required as the compute shader will overwrite the vertex buffer data
    VkBufferMemoryBarrier bufferBarrier = vkTools::initializers::bufferMemoryBarrier();
    // Vertex shader invocations have finished reading from the buffer
    bufferBarrier.srcAccessMask = VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT;
    // Compute shader buffer read and write
    bufferBarrier.dstAccessMask = VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_SHADER_READ_BIT;
    bufferBarrier.buffer = particlesStorageBuffer.buffer;
    bufferBarrier.size = particlesStorageBuffer.descriptor.range;
    bufferBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    bufferBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;

    vkCmdPipelineBarrier(
        cmdbuffer,
        VK_PIPELINE_STAGE_VERTEX_SHADER_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        VK_FLAGS_NONE,
        0, nullptr,
        1, &bufferBarrier,
        0, nullptr);

    vkCmdBindPipeline(cmdbuffer, VK_PIPELINE_BIND_POINT_COMPUTE, propageFire);
    vkCmdBindDescriptorSets(cmdbuffer, VK_PIPELINE_BIND_POINT_COMPUTE, computePipelineLayout, 0, 1, &computeDescriptorSet, 0, 0);

    // Dispatch the compute job
    vkCmdDispatch(cmdbuffer, bGroups, 1, 1);

    vkCmdBindPipeline(cmdbuffer, VK_PIPELINE_BIND_POINT_COMPUTE, updateParticles);
    vkCmdBindDescriptorSets(cmdbuffer, VK_PIPELINE_BIND_POINT_COMPUTE, computePipelineLayout, 0, 1, &computeDescriptorSet, 0, 0);

    // Dispatch the compute job
    vkCmdDispatch(cmdbuffer, pGroups, 1, 1);

    // Add memory barrier to ensure that compute shader has finished writing to the buffer
    // Without this the (rendering) vertex shader may display incomplete results (partial data from last frame)
    // Compute shader has finished writes to the buffer
    bufferBarrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    // Vertex shader access (attribute binding)
    bufferBarrier.dstAccessMask = VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT;
    bufferBarrier.buffer = particlesStorageBuffer.buffer;
    bufferBarrier.size = particlesStorageBuffer.descriptor.range;
    bufferBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    bufferBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;

    vkCmdPipelineBarrier(
        cmdbuffer,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        VK_PIPELINE_STAGE_VERTEX_SHADER_BIT,
        VK_FLAGS_NONE,
        0, nullptr,
        1, &bufferBarrier,
        0, nullptr);
}

void VulkanFire::draw(VkCommandBuffer cmdbuffer)
{
    // Particle system
    vkCmdBindPipeline(cmdbuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, drawParticles);
    vkCmdBindDescriptorSets(cmdbuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSet, 0, NULL);

    VkDeviceSize offsets[1] = { 0 };
    vkCmdBindVertexBuffers(cmdbuffer, 0, 1, &particlesStorageBuffer.buffer, offsets);
    vkCmdDraw(cmdbuffer, computeUbo.particleCount, 1, 0, 0);
}

void VulkanFire::addBurningPoints(std::vector<glm::vec3> data, uint32_t objectNumber){

    int size = data.size();
    std::cout<<size<<std::endl;
    std::vector<glm::vec3> allPoints;
    BurningPoint bp;
    for(int i=0;i<size;i+=4){

        triangulate(data,i,allPoints);
    }

    uint32_t bSize = burningPoints.size();
    for(int i=0;i<allPoints.size();i++){
        bp.pos[0] = bp.basePos[0] = allPoints[i].x;
        bp.pos[1] = bp.basePos[1] = allPoints[i].y;
        bp.pos[2] = bp.basePos[2] = allPoints[i].z;
        bp.basePos[3]=1.0f;
        bp.nCount = 0;
        bp.state = 0;
        for(int j=0;j<allPoints.size();j++){
            if(i!=j){
                glm::vec3 diff = allPoints[j] - allPoints[i];
                float d = glm::dot(diff,diff);
                if(d<0.0121){
                    bp.neighboors[bp.nCount]=j+bSize;
                    bp.nCount++;
                }
                if(bp.nCount==10){
                    break;
                }
            }
        }
        bp.nCount+=objectNumber<<16;
        burningPoints.push_back(bp);
    }
    //burningPoints[0].state=10000;
    std::cout<<"bpointsize "<<burningPoints.size()<<std::endl;
    computeUbo.bPointsCount = burningPoints.size();std::cout<<"bpointsizce "<<computeUbo.bPointsCount<<std::endl;
}
void VulkanFire::triangulate(std::vector<glm::vec3> data,int i,std::vector<glm::vec3>& allPoints){
    Fade_2D dt;
    glm::mat3 mat,matt;
    glm::vec3 u,v,w,a;
    float z;
    std::vector<Point2> vPoints;
    std::vector<Point2*> vPPoints;
    std::vector<Segment2> vSegments1;
    std::vector<Triangle2*> vTriangles2;
    std::vector<ConstraintGraph2*> vCG;

    std::stringstream sstm;
    u = glm::normalize(data.at(i+1)-data.at(i));
    w = data.at(i+3);
    v = glm::cross(u,w);

    mat = glm::mat3();

    mat[0]=u;
    mat[1]=v;
    mat[2]=w;

    matt =glm::transpose(mat);
    for(int j=0;j<3;j++){
        a = matt*data.at(i+j);
        vPoints.push_back(Point2(a.x,a.y));
    }
    z=a.z;
    dt.insert(vPoints);
    for(int j=0;j<3;j++){
        Point2& p0(vPoints[j]);
        Point2& p1(vPoints[(j+1)%3]);
        vSegments1.push_back(Segment2(p0,p1));
    }

    ConstraintGraph2* pCG1=dt.createConstraint(vSegments1,CIS_CONSTRAINED_DELAUNAY);

    vCG.push_back(pCG1);

    Zone2* pZone=dt.createZone(vCG,ZL_GROW,vPoints[0]);
    dt.applyConstraintsAndZones();
    dt.refine(pZone,27,0.01,0.1,true);

    pZone->getTriangles(vTriangles2);

    vPoints.clear();

    dt.getVertexPointers(vPPoints);

    for(int i=0;i<vPPoints.size();i++){
        u = mat*glm::vec3(vPPoints[i]->x(),vPPoints[i]->y(),z);
        if(u.x < -1 || u.y< -1 || u.z<-1){
            std::cout << "point bizartre " << u.x << " " << u.y << " " << u.z << std::endl;
        }
        allPoints.push_back(u);
    }

    dt.deleteZone(pZone);

    vSegments1.clear();
    vCG.clear();
}

void VulkanFire::init(VkQueue queue,VkCommandPool cpool, VkRenderPass renderpass,VkDescriptorPool pool,VkDescriptorBufferInfo* descriptor, VkSampler sampler,vkTools::VulkanTexture fire,vkTools::VulkanTexture smoke, std::string path){
    std::cout<< "dajge" << std::endl;
    prepareParticles(queue);
    prepareBurningPoints(queue);
    prepareUniformBuffers();
std::cout<< "dajge1" << std::endl;
    prepareComputeLayout(pool,descriptor);
    std::cout<< "dajge2" << std::endl;
    prepareComputePipelines(path);
std::cout<< "dajge3" << std::endl;
    setupDescriptorSet(pool,sampler,fire,smoke);
    prepareRenderPipelines(renderpass,path);
    createClickCommand(cpool);
}

float VulkanFire::rnd(float range)
{
    return range * (rand() / double(RAND_MAX));
}

void VulkanFire::initParticle(Particle *particle, int index)
{

    //Body creation
    /*btCollisionShape* groundShape = new btSphereShape(btScalar(particle->size));
    btVector3 localInertia(0,0,0);

    btTransform groundTransform;
    groundTransform.setIdentity();

    VkParticleMotionState* myMotionState = new VkParticleMotionState(groundTransform,this,index);

    groundShape->calculateLocalInertia(PARTICLE_MASS,localInertia);

    btRigidBody::btRigidBodyConstructionInfo rbInfo(PARTICLE_MASS,myMotionState,groundShape,localInertia);
    particle->body = new btRigidBody(rbInfo,false);

    reset(particle);*/

}


void VulkanFire::reset(Particle *particle){
    /*;

    particle->body->getWorldTransform().setOrigin(btVector3(particle->pos.x, particle->pos.y, particle->pos.z));

    particle->body->setLinearVelocity(btVector3(0.0f, minVel.y + rnd(maxVel.y - minVel.y), 0.0f));*/
}

void VulkanFire::transitionParticle(Particle *particle)
{
    /*switch (particle->type)
    {
    case PARTICLE_TYPE_FLAME:
        // Flame particles have a chance of turning into smoke
        if (rnd(1.0f) < 0.05f)
        {
            particle->alpha = 0.0f;
            particle->color = glm::vec4(0.25f + rnd(0.25f));

            particle->body->getWorldTransform().setOrigin(btVector3(particle->pos.x*0.5f, particle->pos.y, particle->pos.z*0.5f));
            particle->body->setLinearVelocity(btVector3(rnd(1.0f) - rnd(1.0f), (minVel.y * 2) + rnd(maxVel.y - minVel.y), rnd(1.0f) - rnd(1.0f)));
            particle->size = 1.0f + rnd(0.5f);
            particle->rotationSpeed = rnd(1.0f) - rnd(1.0f);
            particle->type = PARTICLE_TYPE_SMOKE;
        }
        else
        {
            reset(particle);
        }
        break;
    case PARTICLE_TYPE_SMOKE:
        // Respawn at end of life
        reset(particle);
        break;
    }*/
}

void VulkanFire::prepareParticles(VkQueue queue)
{

    glm::vec3 minVel = glm::vec3(-3.0f, 0.5f, -3.0f);
    glm::vec3 maxVel = glm::vec3(3.0f, 7.0f, 3.0f);
    std::vector<Particle> particleBuffer;
    std::cout<< "debut " <<computeUbo.bPointsCount<< std::endl;
    particleBuffer.resize(computeUbo.bPointsCount);
    computeUbo.particleCount =computeUbo.bPointsCount;
    int row=0;
    int col=0;
    float step=3.0f;
    for (auto& particle : particleBuffer)
    {
        particle.pos[0]=-90+row*step;
        particle.pos[1]=-10;
        particle.pos[2]=-90+col*step;
        particle.alpha = 0.75f;//rnd(0.75f);
        particle.size = 0.05f + rnd(0.025f);
        particle.color = glm::vec4(1.0f);
        particle.type = PARTICLE_TYPE_FLAME;
        particle.rotation = rnd(2.0f * M_PI);
        particle.vel.w = rnd(2.0f) - rnd(2.0f);
        particle.vel=glm::vec4(0.0f, minVel.y + rnd(maxVel.y - minVel.y), 0.0f, 0.0f);

        row++;
        if(row==60){
            row=0;
            col++;
        }
    }


    // Setup and fill the compute shader storage buffers for
    // vertex positions and velocities

    uint32_t storageBufferSize = particleBuffer.size() * sizeof(Particle);

    // Staging
    // SSBO is static, copy to device local memory
    // This results in better performance

    struct {
        VkDeviceMemory memory;
        VkBuffer buffer;
    } stagingBuffer;
std::cout<< "alloc particle " <<sizeof(Particle)<< std::endl;
    exampleBase->createBuffer(
        VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT,
        storageBufferSize,
        particleBuffer.data(),
        &stagingBuffer.buffer,
        &stagingBuffer.memory);
std::cout<< "alloc particle local" <<sizeof(BurningPoint)<< std::endl;
    exampleBase->createBuffer(
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
        storageBufferSize,
        nullptr,
        &particlesStorageBuffer.buffer,
        &particlesStorageBuffer.memory);
std::cout<< "alloc particle end" << std::endl;
    // Copy to staging buffer
    VkCommandBuffer copyCmd = exampleBase->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);

    VkBufferCopy copyRegion = {};
    copyRegion.size = storageBufferSize;
    vkCmdCopyBuffer(
        copyCmd,
        stagingBuffer.buffer,
        particlesStorageBuffer.buffer,
        1,
        &copyRegion);

    exampleBase->flushCommandBuffer(copyCmd, queue, true);

    vkFreeMemory(device, stagingBuffer.memory, nullptr);
    vkDestroyBuffer(device, stagingBuffer.buffer, nullptr);

    particlesStorageBuffer.descriptor.range = storageBufferSize;
    particlesStorageBuffer.descriptor.buffer = particlesStorageBuffer.buffer;
    particlesStorageBuffer.descriptor.offset = 0;

    // Binding description
    attributesParticles.bindingDescriptions.resize(1);
    attributesParticles.bindingDescriptions[0] =
        vkTools::initializers::vertexInputBindingDescription(
            VERTEX_BUFFER_BIND_ID,
            sizeof(Particle),
            VK_VERTEX_INPUT_RATE_VERTEX);

    // Attribute descriptions
    // Describes memory layout and shader positions
    // Location 0 : Position
    attributesParticles.attributeDescriptions.push_back(
        vkTools::initializers::vertexInputAttributeDescription(
            VERTEX_BUFFER_BIND_ID,
            0,
            VK_FORMAT_R32G32B32A32_SFLOAT,
            0));
    // Location 1 : Color
    attributesParticles.attributeDescriptions.push_back(
        vkTools::initializers::vertexInputAttributeDescription(
            VERTEX_BUFFER_BIND_ID,
            1,
            VK_FORMAT_R32G32B32A32_SFLOAT,
            sizeof(float) * 4));
    // Location 2 : Alpha
    attributesParticles.attributeDescriptions.push_back(
        vkTools::initializers::vertexInputAttributeDescription(
            VERTEX_BUFFER_BIND_ID,
            2,
            VK_FORMAT_R32_SFLOAT,
            sizeof(float) * 8));
    // Location 3 : Size
    attributesParticles.attributeDescriptions.push_back(
        vkTools::initializers::vertexInputAttributeDescription(
            VERTEX_BUFFER_BIND_ID,
            3,
            VK_FORMAT_R32_SFLOAT,
            sizeof(float) * 9));
    // Location 4 : Rotation
    attributesParticles.attributeDescriptions.push_back(
        vkTools::initializers::vertexInputAttributeDescription(
            VERTEX_BUFFER_BIND_ID,
            4,
            VK_FORMAT_R32_SFLOAT,
            sizeof(float) * 10));
    // Location 5 : Type
    attributesParticles.attributeDescriptions.push_back(
        vkTools::initializers::vertexInputAttributeDescription(
            VERTEX_BUFFER_BIND_ID,
            5,
            VK_FORMAT_R32_SINT,
            sizeof(float) * 11));

    // Assign to vertex buffer
    attributesParticles.inputState = vkTools::initializers::pipelineVertexInputStateCreateInfo();
    attributesParticles.inputState.vertexBindingDescriptionCount = attributesParticles.bindingDescriptions.size();
    attributesParticles.inputState.pVertexBindingDescriptions = attributesParticles.bindingDescriptions.data();
    attributesParticles.inputState.vertexAttributeDescriptionCount = attributesParticles.attributeDescriptions.size();
    attributesParticles.inputState.pVertexAttributeDescriptions = attributesParticles.attributeDescriptions.data();
}

void VulkanFire::prepareBurningPoints(VkQueue queue){
    // Setup and fill the compute shader storage buffers for
    // vertex positions and velocities

    uint32_t storageBufferSize = computeUbo.bPointsCount * sizeof(BurningPoint);

    // Staging
    // SSBO is static, copy to device local memory
    // This results in better performance

    struct {
        VkDeviceMemory memory;
        VkBuffer buffer;
    } stagingBuffer;
    std::cout<< "alloc burning" << std::endl;
    exampleBase->createBuffer(
        VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT,
        storageBufferSize,
        burningPoints.data(),
        &stagingBuffer.buffer,
        &stagingBuffer.memory);

    exampleBase->createBuffer(
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
        storageBufferSize,
        nullptr,
        &bPointsStorageBuffer.buffer,
        &bPointsStorageBuffer.memory);

    // Copy to staging buffer
    VkCommandBuffer copyCmd = exampleBase->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);

    VkBufferCopy copyRegion = {};
    copyRegion.size = storageBufferSize;
    vkCmdCopyBuffer(
        copyCmd,
        stagingBuffer.buffer,
        bPointsStorageBuffer.buffer,
        1,
        &copyRegion);

    exampleBase->flushCommandBuffer(copyCmd, queue, true);

    vkFreeMemory(device, stagingBuffer.memory, nullptr);
    vkDestroyBuffer(device, stagingBuffer.buffer, nullptr);

    bPointsStorageBuffer.descriptor.range = storageBufferSize;
    bPointsStorageBuffer.descriptor.buffer = bPointsStorageBuffer.buffer;
    bPointsStorageBuffer.descriptor.offset = 0;






    /*
    // Binding description
    attributesBPoints.bindingDescriptions.resize(1);
    attributesBPoints.bindingDescriptions[0] =
        vkTools::initializers::vertexInputBindingDescription(
            VERTEX_BUFFER_BIND_ID,
            sizeof(BurningPoint),
            VK_VERTEX_INPUT_RATE_VERTEX);

    // Attribute descriptions
    // Describes memory layout and shader positions
    // Location 0 : Position
    attributesBPoints.attributeDescriptions.push_back(
        vkTools::initializers::vertexInputAttributeDescription(
            VERTEX_BUFFER_BIND_ID,
            0,
            VK_FORMAT_R32G32B32_SFLOAT,
            0));
    // Location 1 : Voisins 1à3
    attributesBPoints.attributeDescriptions.push_back(
        vkTools::initializers::vertexInputAttributeDescription(
            VERTEX_BUFFER_BIND_ID,
            1,
            VK_FORMAT_R32G32B32_UINT,
            sizeof(float) * 3));
    // Location 2 : Voisins 4 à 6
    attributesBPoints.attributeDescriptions.push_back(
        vkTools::initializers::vertexInputAttributeDescription(
            VERTEX_BUFFER_BIND_ID,
            2,
            VK_FORMAT_R32G32B32_UINT,
            sizeof(unsigned int) * 3+sizeof(float) * 3));
    // Location 3 : nCount
    attributesBPoints.attributeDescriptions.push_back(
        vkTools::initializers::vertexInputAttributeDescription(
            VERTEX_BUFFER_BIND_ID,
            3,
            VK_FORMAT_R32_UINT,
            sizeof(unsigned int) * 6+sizeof(float) * 3));
    // Location 4 : state
    attributesBPoints.attributeDescriptions.push_back(
        vkTools::initializers::vertexInputAttributeDescription(
            VERTEX_BUFFER_BIND_ID,
            4,
            VK_FORMAT_R16_UINT,
            sizeof(unsigned int) * 7+sizeof(float) * 3));

    // Assign to vertex buffer
    attributesBPoints.inputState = vkTools::initializers::pipelineVertexInputStateCreateInfo();
    attributesBPoints.inputState.vertexBindingDescriptionCount = attributesBPoints.bindingDescriptions.size();
    attributesBPoints.inputState.pVertexBindingDescriptions = attributesBPoints.bindingDescriptions.data();
    attributesBPoints.inputState.vertexAttributeDescriptionCount = attributesBPoints.attributeDescriptions.size();
    attributesBPoints.inputState.pVertexAttributeDescriptions = attributesBPoints.attributeDescriptions.data();
    */
}

void VulkanFire::prepareComputeLayout(VkDescriptorPool descriptorPool, VkDescriptorBufferInfo* descriptor)
{
    // Create compute pipeline
    // Compute pipelines are created separate from graphics pipelines
    // even if they use the same queue

    std::vector<VkDescriptorSetLayoutBinding> setLayoutBindings = {
        // Binding 0 : Particle position storage buffer
        vkTools::initializers::descriptorSetLayoutBinding(
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
            VK_SHADER_STAGE_COMPUTE_BIT,
            0),
        vkTools::initializers::descriptorSetLayoutBinding(
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
            VK_SHADER_STAGE_COMPUTE_BIT,
            1),
        // Binding 1 : Uniform buffer
        vkTools::initializers::descriptorSetLayoutBinding(
            VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
            VK_SHADER_STAGE_COMPUTE_BIT,
            2),
    };

    VkDescriptorSetLayoutCreateInfo descriptorLayout =
        vkTools::initializers::descriptorSetLayoutCreateInfo(
            setLayoutBindings.data(),
            setLayoutBindings.size());

    VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device,	&descriptorLayout, nullptr,	&computeDescriptorSetLayout));


    VkPipelineLayoutCreateInfo pPipelineLayoutCreateInfo =
        vkTools::initializers::pipelineLayoutCreateInfo(
            &computeDescriptorSetLayout,
            1);

    VK_CHECK_RESULT(vkCreatePipelineLayout(device, &pPipelineLayoutCreateInfo, nullptr,	&computePipelineLayout));

    VkDescriptorSetAllocateInfo allocInfo =
        vkTools::initializers::descriptorSetAllocateInfo(
            descriptorPool,
            &computeDescriptorSetLayout,
            1);

    VK_CHECK_RESULT(vkAllocateDescriptorSets(device, &allocInfo, &computeDescriptorSet));

    std::vector<VkWriteDescriptorSet> computeWriteDescriptorSets =
    {
        // Binding 0 : Particle position storage buffer
        vkTools::initializers::writeDescriptorSet(
            computeDescriptorSet,
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
            0,
            &particlesStorageBuffer.descriptor),
        // Binding 0 : burning points storage buffer
        vkTools::initializers::writeDescriptorSet(
            computeDescriptorSet,
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
            1,
            &bPointsStorageBuffer.descriptor),
        // Binding 1 : Uniform buffer
        vkTools::initializers::writeDescriptorSet(
            computeDescriptorSet,
            VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
            2,
            &computeUniformBuffer.descriptor)
    };
    vkUpdateDescriptorSets(device, computeWriteDescriptorSets.size(), computeWriteDescriptorSets.data(), 0, NULL);

    VK_CHECK_RESULT(vkAllocateDescriptorSets(device, &allocInfo, &moveBurnDescriptorSet));

    computeWriteDescriptorSets =
    {
        vkTools::initializers::writeDescriptorSet(
            moveBurnDescriptorSet,
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
            1,
            &bPointsStorageBuffer.descriptor),
        // Binding 1 : Uniform buffer
        vkTools::initializers::writeDescriptorSet(
            moveBurnDescriptorSet,
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
            0,
            descriptor)
    };
    vkUpdateDescriptorSets(device, computeWriteDescriptorSets.size(), computeWriteDescriptorSets.data(), 0, NULL);

}

void VulkanFire::prepareComputePipelines(std::string assetPath){

    VkPipelineCache pipelineCache;

    VkPipelineCacheCreateInfo pipelineCacheCreateInfo = {};
    pipelineCacheCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_CACHE_CREATE_INFO;
    VK_CHECK_RESULT(vkCreatePipelineCache(device, &pipelineCacheCreateInfo, nullptr, &pipelineCache));


    VkComputePipelineCreateInfo computePipelineCreateInfo =
        vkTools::initializers::computePipelineCreateInfo(
            computePipelineLayout,
            0);

    // Create upadate particle pipeline
    computePipelineCreateInfo.stage = exampleBase->loadShader(assetPath + "shaders/particlefire/updateParticle.comp.spv", VK_SHADER_STAGE_COMPUTE_BIT);
    VK_CHECK_RESULT(vkCreateComputePipelines(device, pipelineCache, 1, &computePipelineCreateInfo, nullptr, &updateParticles));

    computePipelineCreateInfo.stage = exampleBase->loadShader(assetPath + "shaders/particlefire/propageFire.comp.spv", VK_SHADER_STAGE_COMPUTE_BIT);
    VK_CHECK_RESULT(vkCreateComputePipelines(device, pipelineCache, 1, &computePipelineCreateInfo, nullptr, &propageFire));

    computePipelineCreateInfo.stage = exampleBase->loadShader(assetPath + "shaders/particlefire/clickFire.comp.spv", VK_SHADER_STAGE_COMPUTE_BIT);
    VK_CHECK_RESULT(vkCreateComputePipelines(device, pipelineCache, 1, &computePipelineCreateInfo, nullptr, &clickFire));

    computePipelineCreateInfo.stage = exampleBase->loadShader(assetPath + "shaders/particlefire/moveBurn.comp.spv", VK_SHADER_STAGE_COMPUTE_BIT);
    VK_CHECK_RESULT(vkCreateComputePipelines(device, pipelineCache, 1, &computePipelineCreateInfo, nullptr, &moveBurnPipeline));
}

void VulkanFire::createClickCommand(VkCommandPool cmdPool){

    int bGroups  = 1;
    while(bGroups*512<computeUbo.bPointsCount){
        bGroups++;
    }
    VkCommandBufferAllocateInfo cmdBufAllocateInfo =
        vkTools::initializers::commandBufferAllocateInfo(
            cmdPool,
            VK_COMMAND_BUFFER_LEVEL_PRIMARY,
            1);

    VK_CHECK_RESULT(vkAllocateCommandBuffers(device, &cmdBufAllocateInfo, &clickCmd));

    VkCommandBufferBeginInfo cmdBufInfo = vkTools::initializers::commandBufferBeginInfo();

    vkBeginCommandBuffer(clickCmd,&cmdBufInfo);

    vkCmdBindPipeline(clickCmd, VK_PIPELINE_BIND_POINT_COMPUTE, clickFire);
    vkCmdBindDescriptorSets(clickCmd, VK_PIPELINE_BIND_POINT_COMPUTE, computePipelineLayout, 0, 1, &computeDescriptorSet, 0, 0);

    // Dispatch the compute job
    std::cout<<"click"<<bGroups<<std::endl;
    vkCmdDispatch(clickCmd, bGroups, 1, 1);

    vkEndCommandBuffer(clickCmd);
}

void VulkanFire::buildMoveBurnCommand(VkCommandBuffer& cmd){
    int bGroups  = 1;
    while(bGroups*512<computeUbo.bPointsCount){
        bGroups++;
    }
    VkCommandBufferBeginInfo cmdBufInfo = vkTools::initializers::commandBufferBeginInfo();

    vkBeginCommandBuffer(cmd,&cmdBufInfo);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, moveBurnPipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, computePipelineLayout, 0, 1, &moveBurnDescriptorSet, 0, 0);

    // Dispatch the compute job
    std::cout<<"click"<<bGroups<<std::endl;
    vkCmdDispatch(cmd, bGroups, 1, 1);

    vkEndCommandBuffer(cmd);
}

void VulkanFire::cliked(VkQueue queue, glm::vec4 pos){
    computeUbo.clickPos = pos;

    memcpy(computeUniformBuffer.mapped, &computeUbo, sizeof(computeUbo));

    VkSubmitInfo submitInfo = vkTools::initializers::submitInfo();
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &clickCmd;

    // Submit to queue
    std::cout<<"click"<<std::endl;
    vkDeviceWaitIdle(device);
    vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
    std::cout<<"click2"<<std::endl;
}

void VulkanFire::prepareRenderLayout(VkDescriptorPool descriptorPool){
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
        // Binding 2 : Fragment shader image sampler
        vkTools::initializers::descriptorSetLayoutBinding(
            VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
            VK_SHADER_STAGE_FRAGMENT_BIT,
            2)
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

void VulkanFire::prepareRenderPipelines(VkRenderPass renderPass,std::string assetPath){

    VkPipelineInputAssemblyStateCreateInfo inputAssemblyState =
        vkTools::initializers::pipelineInputAssemblyStateCreateInfo(
            VK_PRIMITIVE_TOPOLOGY_POINT_LIST,
            0,
            VK_FALSE);

    VkPipelineRasterizationStateCreateInfo rasterizationState =
        vkTools::initializers::pipelineRasterizationStateCreateInfo(
            VK_POLYGON_MODE_FILL,
            VK_CULL_MODE_BACK_BIT,
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
        VK_DYNAMIC_STATE_SCISSOR
    };
    VkPipelineDynamicStateCreateInfo dynamicState =
        vkTools::initializers::pipelineDynamicStateCreateInfo(
            dynamicStateEnables.data(),
            dynamicStateEnables.size(),
            0);

    // Load shaders
    std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages;

    shaderStages[0] = exampleBase->loadShader(assetPath + "shaders/particlefire/particle.vert.spv", VK_SHADER_STAGE_VERTEX_BIT);
    shaderStages[1] = exampleBase->loadShader(assetPath + "shaders/particlefire/particle.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);

    VkGraphicsPipelineCreateInfo pipelineCreateInfo =
        vkTools::initializers::pipelineCreateInfo(
            pipelineLayout,
            renderPass,
            0);

    pipelineCreateInfo.pVertexInputState = &attributesParticles.inputState;
    pipelineCreateInfo.pInputAssemblyState = &inputAssemblyState;
    pipelineCreateInfo.pRasterizationState = &rasterizationState;
    pipelineCreateInfo.pColorBlendState = &colorBlendState;
    pipelineCreateInfo.pMultisampleState = &multisampleState;
    pipelineCreateInfo.pViewportState = &viewportState;
    pipelineCreateInfo.pDepthStencilState = &depthStencilState;
    pipelineCreateInfo.pDynamicState = &dynamicState;
    pipelineCreateInfo.stageCount = shaderStages.size();
    pipelineCreateInfo.pStages = shaderStages.data();

    depthStencilState.depthWriteEnable = VK_FALSE;

    // Premulitplied alpha
    blendAttachmentState.blendEnable = VK_TRUE;
    blendAttachmentState.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
    blendAttachmentState.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    blendAttachmentState.colorBlendOp = VK_BLEND_OP_ADD;
    blendAttachmentState.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    blendAttachmentState.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
    blendAttachmentState.alphaBlendOp = VK_BLEND_OP_ADD;
    blendAttachmentState.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;

    VkPipelineCache pipelineCache;

    VkPipelineCacheCreateInfo pipelineCacheCreateInfo = {};
    pipelineCacheCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_CACHE_CREATE_INFO;
    VK_CHECK_RESULT(vkCreatePipelineCache(device, &pipelineCacheCreateInfo, nullptr, &pipelineCache));

    VkResult err = vkCreateGraphicsPipelines(device, pipelineCache, 1, &pipelineCreateInfo, nullptr, &drawParticles);
    assert(!err);
}


/*void VulkanFire::updateParticle(btVector3 ori, int index)
{
    Particle* particle = &particleBuffer.at(index);
    float particleTimer = 0.0003 * 0.45f;
    switch (particle->type)
    {
    case PARTICLE_TYPE_FLAME:
        particle->pos =glm::vec4(ori.x(),ori.y(),ori.z(),0.0f);
        particle->alpha += particleTimer * 2.5f;
        particle->size -= particleTimer * 0.5f;
        break;
    case PARTICLE_TYPE_SMOKE:
        particle->alpha += particleTimer * 1.25f;
        particle->size += particleTimer * 0.125f;
        particle->color -= particleTimer * 0.05f;
        break;
    }
    particle->rotation += particleTimer * particle->rotationSpeed;
    // Transition particle state
    if (particle->alpha > 2.0f)
    {
        transitionParticle(particle);
    }
    size_t size = particleBuffer.size() * sizeof(Particle);
    memcpy(particleVkBuffer.mappedMemory, particleBuffer.data(), size);
}*/

void VulkanFire::setupDescriptorSet(VkDescriptorPool pool,VkSampler sampler,vkTools::VulkanTexture fire,vkTools::VulkanTexture smoke)
{
    std::cout<<"bobo" << std::endl;
    prepareRenderLayout(pool);
    std::cout<<"bobo2" << std::endl;
    VkDescriptorSetAllocateInfo allocInfo =
        vkTools::initializers::descriptorSetAllocateInfo(
            pool,
            &descriptorSetLayout,
            1);
std::cout<<"bobo3" << std::endl;
    VkResult vkRes = vkAllocateDescriptorSets(device, &allocInfo, &descriptorSet);
    assert(!vkRes);
std::cout<<"bobo4" << std::endl;
    // Image descriptor for the color map texture
    VkDescriptorImageInfo texDescriptorSmoke =
        vkTools::initializers::descriptorImageInfo(
            sampler,
            smoke.view,
            VK_IMAGE_LAYOUT_GENERAL);
    VkDescriptorImageInfo texDescriptorFire =
        vkTools::initializers::descriptorImageInfo(
            sampler,
            fire.view,
            VK_IMAGE_LAYOUT_GENERAL);

    std::vector<VkWriteDescriptorSet> writeDescriptorSets =
    {
        // Binding 0 : Vertex shader uniform buffer
        vkTools::initializers::writeDescriptorSet(
        descriptorSet,
            VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
            0,
            &uniformData.descriptor),
        // Binding 1 : Smoke texture
        vkTools::initializers::writeDescriptorSet(
            descriptorSet,
            VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
            1,
            &texDescriptorSmoke),
        // Binding 1 : Fire texture array
        vkTools::initializers::writeDescriptorSet(
            descriptorSet,
            VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
            2,
            &texDescriptorFire)
    };

    vkUpdateDescriptorSets(device, writeDescriptorSets.size(), writeDescriptorSets.data(), 0, NULL);
}

void VulkanFire::prepareUniformBuffers()
{
    // Vertex shader uniform buffer block
    std::cout<< "alloc uniform" << std::endl;

    exampleBase->createBuffer(
        VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
        sizeof(ubo),
        &ubo,
        &uniformData.buffer,
        &uniformData.memory,
        &uniformData.descriptor);


    //computeUbo.deltaT=0.0f;

    std::cout<< "alloc cuniform" << std::endl;
    // Compute shader uniform buffer block
    exampleBase->createBuffer(
        VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
        sizeof(computeUbo),
        &computeUbo,
        &computeUniformBuffer.buffer,
        &computeUniformBuffer.memory,
        &computeUniformBuffer.descriptor);

    // Map for host access
    vkTools::checkResult(vkMapMemory(device, computeUniformBuffer.memory, 0, sizeof(computeUbo), 0, (void **)&computeUniformBuffer.mapped));
}


void VulkanFire::updateTime(float frameTimer)
{
    /*computeUbo.deltaT = frameTimer * -0.5f;*/
    //computeUbo.bPointsCount = 2;
    computeUbo.delta= glm::vec4(1,6,1,frameTimer);

    computeUbo.clickPos = glm::vec4(5,2,3,4);
    //std::cout<< sizeof(computeUbo) << std::endl;
    memcpy(computeUniformBuffer.mapped, &computeUbo, sizeof(computeUbo));
}

void VulkanFire::addToWorld(btDiscreteDynamicsWorld* dw){
    /*for (auto& particle : particleBuffer)
    {
        dw->addRigidBody(particle.body);
        particle.body->setGravity(btVector3(0.0f,0.0f,0.0f));
    }*/
}

void VulkanFire::updateProjView(glm::mat4 projection, glm::mat4 view){
    ubo.projection = projection;
    ubo.view = view;
    updateUniformBuffer();
}

void VulkanFire::updateUniformBuffer(){
    uint8_t *pData;
    VkResult err = vkMapMemory(device, uniformData.memory, 0, sizeof(ubo), 0, (void **)&pData);
    assert(!err);
    memcpy(pData, &ubo, sizeof(ubo));
    vkUnmapMemory(device, uniformData.memory);
}
