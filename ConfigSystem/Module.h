/*! @file Module.h
    @brief Test configurable module class.
    
    @author Mitchell Metcalfe
*/

#ifndef Module_H
#define Module_H

#include "Configurable.h"
#include "ConfigManager.h"
extern ConfigSystem::ConfigManager* config;

class Module : public Configurable
{
public:
    typedef std::vector<std::vector<std::vector<double> > > vec3D_d;
    typedef std::vector<std::vector<std::vector<long  > > > vec3D_l;
    typedef std::vector<std::vector<double> > vec2D_d;
    typedef std::vector<std::vector<long  > > vec2D_l;
    typedef std::vector<double> vec1D_d;
    typedef std::vector<long  > vec1D_l;
    vec3D_d     val_3dv_d, val_3dv_d_new; bool val_changed_3dv_d;
    vec3D_l     val_3dv_l, val_3dv_l_new; bool val_changed_3dv_l;
    vec2D_d     val_2dv_d, val_2dv_d_new; bool val_changed_2dv_d;
    vec2D_l     val_2dv_l, val_2dv_l_new; bool val_changed_2dv_l;
    vec1D_d     val_1dv_d, val_1dv_d_new; bool val_changed_1dv_d;
    vec1D_l     val_1dv_l, val_1dv_l_new; bool val_changed_1dv_l;
    double      val_d    , val_d_new    ; bool val_changed_d    ;
    long        val_l    , val_l_new    ; bool val_changed_l    ;
    std::string val_s    , val_s_new    ; bool val_changed_s    ;


    std::string name_3dv_d;
    std::string name_3dv_l;
    std::string name_2dv_d;
    std::string name_2dv_l;
    std::string name_1dv_d;
    std::string name_1dv_l;
    std::string name_d    ;
    std::string name_l    ;
    std::string name_s    ;

    // true if this instance's loadConfig() has not yet been called.
    bool _firstLoad;
    
    bool _shouldUpdate; // whether this module needs to update
    bool _updateCalled; // whether update was called

    //! Implement required methods
    Module();
    void loadConfig();
    void updateConfig();

    /*! Compare old to new + check that vars that were meant to change did so. 
        Prints results.
    */
    bool compare();

    /*! Randomly selects variables in this class and changes their values */
    void change();

    // Methods to change one of this classes member variables in the
    // ConfigSystem, + to set its val_changed_* to indicate the change.
    void change_3dv_d();
    void change_3dv_l();
    void change_2dv_d();
    void change_2dv_l();
    void change_1dv_d();
    void change_1dv_l();
    void change_d    ();
    void change_l    ();
    void change_s    ();

    static bool autoUpdateTest();

};

#endif

