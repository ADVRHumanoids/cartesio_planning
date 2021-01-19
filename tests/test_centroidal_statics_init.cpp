#include <gtest/gtest.h>
#include <validity_checker/stability/centroidal_statics.h>

class TestCentroidalStatics: public ::testing::Test {


protected:

     TestCentroidalStatics(){
         // get model
         std::string config_path = TEST_CONFIG_PATH;
         XBot::ConfigOptions opt;
         opt.set_urdf_path(config_path + "cogimon.urdf");
         opt.set_srdf_path(config_path + "cogimon.srdf");
         opt.generate_jidmap();
         opt.set_parameter("is_model_floating_base", true);
         opt.set_parameter<std::string>("model_type", "RBDL");
         auto model = XBot::ModelInterface::getModel(opt);

         std::vector<std::string> contact_links = {"l_sole", "r_sole", "RSoftHand"};

        cs = std::make_shared<XBot::Cartesian::Planning::CentroidalStatics>(model, contact_links, 0.75);
     }

     virtual ~TestCentroidalStatics() {
     }

     virtual void SetUp() {

     }

     virtual void TearDown() {
     }

public:
    XBot::Cartesian::Planning::CentroidalStatics::Ptr cs;

};

TEST_F(TestCentroidalStatics, testInit)
{
    EXPECT_FALSE(this->cs == NULL);

    std::vector<std::string> contact_links_even = {"l_sole", "r_sole", "RSoftHand", "LSoftHand"};
    std::vector<std::string> contact_links_odd = {"l_sole", "r_sole", "RSoftHand"};
    for(unsigned int i = 0; i < 100; ++i)
    {
        if(i%2 == 0)
            this->cs->setContactLinks(contact_links_even);
        else
            this->cs->setContactLinks(contact_links_odd);
        this->cs->init(false);
        this->cs->checkStability();
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
