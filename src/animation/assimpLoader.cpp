#include "animation/assimpLoader.h"

AssimpLoader::AssimpLoader(const std::string &file)
{

    loadFile(file);

}

void AssimpLoader::loadFile(const std::string &file){
    importer.SetPropertyInteger(AI_CONFIG_PP_LBW_MAX_WEIGHTS, 4);

    int flags = aiProcess_Triangulate;
    flags |= aiProcess_JoinIdenticalVertices;
    //    flags |= aiProcess_GenSmoothNormals;
    flags |= aiProcess_GenUVCoords;
    flags |= aiProcess_TransformUVCoords;
    //    flags |= aiProcess_RemoveComponent;
    flags |= aiProcess_LimitBoneWeights;
    flags |= aiProcess_GenNormals;

    scene = importer.ReadFile( file,flags);
    // If the import failed, report it
    if( !scene)
    {
        cout<< importer.GetErrorString()<<endl;
        exit(0);
    }

    cout<<"Loaded file with assimp2 "<<file<<endl;
    cout<<"HasAnimations "<<scene->mNumAnimations<<
          ", HasCameras "<<scene->mNumCameras<<
          ", HasLights "<<scene->mNumLights<<
          ", HasMaterials "<<scene->mNumMaterials<<
          ", HasMeshes "<<scene->mNumMeshes<<
          ", HasTextures "<<scene->mNumTextures<<endl;

}

void AssimpLoader::loadBones(){
    for(int m =0;m<scene->mNumMeshes;++m){
        const aiMesh *mesh = scene->mMeshes[m];
        for(unsigned int i=0;i<mesh->mNumBones;++i){
            aiBone* b = mesh->mBones[i];

            std::string str(b->mName.data);
            if(boneMap.find(str)==boneMap.end()){

                mat4 boneOffset = convert(b->mOffsetMatrix);
                boneOffsets.push_back(boneOffset);
                boneMap[str] = boneCount++;

            }
        }
    }

    cout<<"unique bones: "<<boneCount<<endl;

    nodeCount = countNodes(scene->mRootNode,rootNode);
    cout<<"unique nodes: "<<nodeCount<<endl;
}

void AssimpLoader::getAnimation(int animationId, int meshId, Animation &out)
{

    const aiMesh *mesh = scene->mMeshes[meshId];

    out.boneMatrices.resize(boneCount);
//    transformmesh(mesh,out.boneMatrices);


    aiAnimation *curanim = scene->mAnimations[animationId];

//    createFrames(mesh,curanim,out.animationFrames);
    createKeyFrames(mesh,curanim,out.animationFrames);

    out.frameCount = out.animationFrames.size();
    out.name = curanim->mName.data;

    cout<<"loaded animation "<<out.name<<": "<<out.frameCount<<" frames"<<endl;
}

void AssimpLoader::transformmesh(const aiMesh *mesh, std::vector<mat4> &boneMatrices)
{
    aiMatrix4x4 skin4;


    //    boneMatrices.resize(mesh->mNumBones);

    for (unsigned int k = 0; k < mesh->mNumBones; k++) {
        aiBone *bone = mesh->mBones[k];
        aiNode *node = findnode(scene->mRootNode, bone->mName.data);

        transformnode(&skin4, node);



        std::string str(bone->mName.data);

        AnimationNode* an = nodeMap[str];

        cout<<"test"<<endl;
        cout<<an->transformedMatrix<<endl<<convert(skin4)<<endl;

        aiMultiplyMatrix4(&skin4, &bone->mOffsetMatrix);


        int index = boneMap[str];
        boneMatrices[index] = convert(skin4);

        //        cout<<"transform matrix "<<index<<endl;


    }

}


void AssimpLoader::createKeyFrames(const aiMesh *mesh, aiAnimation *anim, std::vector<AnimationFrame> &animationFrames)
{
    aiVectorKey *p0, *p1, *s0, *s1;
    aiQuatKey *r0, *r1;
    aiVector3D p, s;
    aiQuaternion r;


    //the last frame is the same as the first
    int frames = animationlength(anim);
//    frames = 1;

    animationFrames.resize(frames);

    float tick =0;

    for(int j=0;j<frames;++j){
        rootNode.reset();
        int frame = j;
        AnimationFrame &k = animationFrames[j];

//        cout<<">>>>>>>>>>>Keyframe "<<frame<<" channels "<<anim->mNumChannels<<endl;


        for (int i = 0; i < anim->mNumChannels; i++) {
            aiNodeAnim *chan = anim->mChannels[i];
            p0 = chan->mPositionKeys + frame;
            r0 = chan->mRotationKeys + frame;
            s0 = chan->mScalingKeys + frame;


            p = p0->mValue;;
            r = r0->mValue;
            s = s0->mValue;

            std::string str(chan->mNodeName.data);
            if(nodeMap.find(str)==nodeMap.end()){
                assert(0);
            }
            AnimationNode* an = nodeMap[str];

            an->position = vec3(p.x,p.y,p.z);
            an->rotation = quat(r.w,r.x,r.y,r.z);
            an->scaling = vec3(s.x,s.y,s.z);
            an->keyFramed = true;
        }

        k.nodeCount = nodeCount;
        k.bones = boneCount;
        k.boneMatrices.resize(boneCount);
        k.rootNode = rootNode;
        k.boneOffsets = boneOffsets;

        k.initTree();
        k.calculateFromTree();
    }
}

void AssimpLoader::createFrames(const aiMesh *mesh, aiAnimation *anim, std::vector<AnimationFrame> &animationFrames)
{
    aiVectorKey *p0, *p1, *s0, *s1;
    aiQuatKey *r0, *r1;
    aiVector3D p, s;
    aiQuaternion r;


    int up = 20;
    int frames = (animationlength(anim)-1) * up + 1 ;
    float delta = 1.0f/up;

    //    frames = animationlength(anim);
    //    delta = 1.0f;

    animationFrames.resize(frames);

    float tick =0;

    for(int j=0;j<frames;++j){

        AnimationFrame &k = animationFrames[j];

        int frame = floor(tick);
        float t = tick - floor(tick);


        for (int i = 0; i < anim->mNumChannels; i++) {
            aiNodeAnim *chan = anim->mChannels[i];
            aiNode *node = findnode(scene->mRootNode, chan->mNodeName.data);
            p0 = chan->mPositionKeys + (frame+0) % chan->mNumPositionKeys;
            p1 = chan->mPositionKeys + (frame+1) % chan->mNumPositionKeys;
            r0 = chan->mRotationKeys + (frame+0) % chan->mNumRotationKeys;
            r1 = chan->mRotationKeys + (frame+1) % chan->mNumRotationKeys;
            s0 = chan->mScalingKeys + (frame+0) % chan->mNumScalingKeys;
            s1 = chan->mScalingKeys + (frame+1) % chan->mNumScalingKeys;


            p = p0->mValue*(1.0f-t) + p1->mValue*t;
            aiQuaternion::Interpolate(r, r0->mValue, r1->mValue, t);
            s = s0->mValue*(1.0f-t) + s1->mValue*t;

            composematrix(&node->mTransformation, &p, &r, &s);


        }

        std::vector<mat4> boneMatrices;
        boneMatrices.resize(boneCount);
        //        transformmesh(mesh,boneMatrices);
        for(int m =0;m<scene->mNumMeshes;++m){
            const aiMesh *mesh = scene->mMeshes[m];
            transformmesh(mesh,boneMatrices);
            //                    cout<<">>"<<endl;
        }

        //                cout<<"============================================================"<<endl;


//        k.setBoneDeformation(boneMatrices);

        tick += delta;
    }
}


//========================= Assimp helper functions ==================================




int AssimpLoader::animationlength(aiAnimation *anim)
{
    unsigned int i, len = 0;
    for (i = 0; i < anim->mNumChannels; i++) {
        struct aiNodeAnim *chan = anim->mChannels[i];
        len = glm::max(len, chan->mNumPositionKeys);
        len = glm::max(len, chan->mNumRotationKeys);
        len = glm::max(len, chan->mNumScalingKeys);
    }
    return len;
}



aiNode *AssimpLoader::findnode(struct aiNode *node, char *name)
{
    int i;
    if (!strcmp(name, node->mName.data))
        return node;
    for (i = 0; i < node->mNumChildren; i++) {
        struct aiNode *found = findnode(node->mChildren[i], name);
        if (found)
            return found;
    }
    return NULL;
}

int AssimpLoader::countNodes(struct aiNode *node, AnimationNode& an)
{
//    cout<<"node "<<node->mName.data<<endl;
    int n = 1;

    int index = 0;
    std::string str(node->mName.data);
    if(nodeMap.find(str)==nodeMap.end()){
        index = nodeMap.size();
        nodeMap[str] = &an;

    }else{
        assert(0);
    }


    if(boneMap.find(str)!=boneMap.end()){
        an.boneIndex = boneMap[str];
    }else{
        an.boneIndex = -1;
    }

    an.index = index;
    an.matrix = convert(node->mTransformation);
    an.name =str;
    an.children.resize(node->mNumChildren);

    for (int i = 0; i < node->mNumChildren; i++) {
        n += countNodes(node->mChildren[i],an.children[i]);
    }
    return n;
}

// calculate absolute transform for node to do mesh skinning
void AssimpLoader::transformnode(aiMatrix4x4 *result, aiNode *node)
{
    //    cout<<"transform "<<node->mName.data<<endl;
    if (node->mParent) {
        transformnode(result, node->mParent);
        aiMultiplyMatrix4(result, &node->mTransformation);
    } else {
        *result = node->mTransformation;
    }

    std::string str(node->mName.data);
    AnimationNode* an = nodeMap[str];
    an->testMat = convert(*result);
}


mat4 AssimpLoader::composematrix(vec3 position, quat rotation, vec3 scaling){
    glm::mat4 t = glm::translate(glm::mat4(),position);
    glm::mat4 r = glm::mat4_cast(rotation);
    glm::mat4 s = glm::scale(glm::mat4(),scaling);



    glm::mat4 erg = t*s*r;


    return erg;
}

void AssimpLoader::composematrix(aiMatrix4x4 *m,
                                 aiVector3D *t, aiQuaternion *q, aiVector3D *s)
{
    // quat to rotation matrix
    m->a1 = 1 - 2 * (q->y * q->y + q->z * q->z);
    m->a2 = 2 * (q->x * q->y - q->z * q->w);
    m->a3 = 2 * (q->x * q->z + q->y * q->w);
    m->b1 = 2 * (q->x * q->y + q->z * q->w);
    m->b2 = 1 - 2 * (q->x * q->x + q->z * q->z);
    m->b3 = 2 * (q->y * q->z - q->x * q->w);
    m->c1 = 2 * (q->x * q->z - q->y * q->w);
    m->c2 = 2 * (q->y * q->z + q->x * q->w);
    m->c3 = 1 - 2 * (q->x * q->x + q->y * q->y);

    // scale matrix
    m->a1 *= s->x; m->a2 *= s->x; m->a3 *= s->x;
    m->b1 *= s->y; m->b2 *= s->y; m->b3 *= s->y;
    m->c1 *= s->z; m->c2 *= s->z; m->c3 *= s->z;

    // set translation
    m->a4 = t->x; m->b4 = t->y; m->c4 = t->z;

    m->d1 = 0; m->d2 = 0; m->d3 = 0; m->d4 = 1;
}

mat4 AssimpLoader::convert(aiMatrix4x4 mat){
    mat4 ret;
    //    mat[0];
    for(int i=0;i<4;++i){
        for(int j=0;j<4;++j){
            ret[i][j] = mat[j][i];
        }
    }
    return ret;
}


