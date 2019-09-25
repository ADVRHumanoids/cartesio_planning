#include "self_collision_checker.h"
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <urdf/model.h>


using namespace XBot::Cartesian::Planning;

SelfCollisionChecker::SelfCollisionChecker(XBot::ModelInterface& model):
    _model(model)
{
    urdf::Model _urdf_model;
    _urdf_model.initString(model.getUrdfString());

    std::vector<boost::shared_ptr<urdf::Link> > links;
    _urdf_model.getLinks ( links );
    typedef std::vector<boost::shared_ptr<urdf::Link> >::iterator it_type;

    for ( it_type iterator = links.begin(); iterator != links.end(); iterator++ )
    {
        boost::shared_ptr<urdf::Link> link = *iterator;

        if ( link->collision )
        {
            if ( link->collision->geometry->type == urdf::Geometry::MESH )
            {
                std::cout << "adding mesh for " << link->name << std::endl;

                boost::shared_ptr< ::urdf::Mesh> collisionGeometry =
                        boost::dynamic_pointer_cast< ::urdf::Mesh> ( link->collision->geometry );

                shapes::Mesh *mesh = shapes::createMeshFromResource ( collisionGeometry->filename );
                if ( mesh == NULL )
                {
                    std::cout << "Error loading mesh for link " << link->name << std::endl;
                    continue;
                }

                std::vector<fcl::Vector3<double>> vertices;
                std::vector<fcl::Triangle> triangles;

                for ( unsigned int i=0; i < mesh->vertex_count; ++i )
                {
                    fcl::Vector3<double> v ( mesh->vertices[3*i]*collisionGeometry->scale.x,
                                             mesh->vertices[3*i + 1]*collisionGeometry->scale.y,
                                             mesh->vertices[3*i + 2]*collisionGeometry->scale.z );

                    vertices.push_back ( v );
                }

                for ( unsigned int i=0; i< mesh->triangle_count; ++i )
                {
                    fcl::Triangle t ( mesh->triangles[3*i],
                                      mesh->triangles[3*i + 1],
                                      mesh->triangles[3*i + 2] );
                    triangles.push_back ( t );
                }

                // add the mesh data into the BVHModel structure
                shared_ptr<fcl::CollisionGeometry<double>> shape;
                shape.reset ( new fcl::BVHModel<fcl::OBBRSS<double>> );
                fcl::BVHModel<fcl::OBBRSS<double>>* bvhModel = ( fcl::BVHModel<fcl::OBBRSS<double>>* ) shape.get();
                bvhModel->beginModel();
                bvhModel->addSubModel ( vertices, triangles );
                bvhModel->endModel();


                boost::shared_ptr<fcl::CollisionObject<double>> collision_object (
                    new fcl::CollisionObject<double> ( shape ) );

                SelfCollisionChecker::collision_info cinfo;
                cinfo.collision_object = collision_object;
                cinfo.shape = shape;
                /* Store the transformation of the CollisionShape from URDF
                 * that is, we store link_T_shape for the actual link */
                cinfo.link_T_shape = toKdl ( link->collision->origin );

                _collision_infos[link->name] = cinfo;

                _links_to_update.insert(link->name); ///TODO: only for link in acm!
            }
            else
            {
                std::cout<<"Skipping shape of none type MESH for link "<<link->name<<std::endl;
            }
        }
        else
            std::cout << "Collision not defined for link " << link->name << std::endl;
    }
}

void SelfCollisionChecker::updateCollisionObjects()
{
    typedef std::set<std::string>::iterator it_links;
    typedef std::map<std::string,boost::shared_ptr<fcl::CollisionObject<double>> >::iterator it_co;

    for ( it_links it = _links_to_update.begin(); it != _links_to_update.end(); ++it )
    {
        std::string link_name = *it;
        KDL::Frame w_T_link, w_T_shape;
        _model.getPose ( link_name, w_T_link );
        w_T_shape = w_T_link * _collision_infos[link_name].link_T_shape;

        fcl::Transform3<double> fcl_w_T_shape = tofcl ( w_T_shape );
        fcl::CollisionObject<double>* collObj_shape = _collision_infos[link_name].collision_object.get();
        collObj_shape->setTransform ( fcl_w_T_shape );
    }
}

bool SelfCollisionChecker::inCollision(const std::string& link1, const std::string& link2)
{
    if(_model.getLinkID(link1) == -1 || _model.getLinkID(link2) == -1)
        return false;

    fcl::CollisionRequest<double> request;

    fcl::CollisionResult<double> result;

    size_t number_of_contacts = fcl::collide<double>(_collision_infos[link1].collision_object.get(),
                                                     _collision_infos[link2].collision_object.get(),
                                                     request, result);

    if(number_of_contacts > 0)
        return true;
    return false;
}
