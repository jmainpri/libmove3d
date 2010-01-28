IF(ANIMATION)
SET(BM3D_MODULE_NAME animation)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME}/proto)
BM3D_SRC_SUBDIR_PROCESS(        anim_interface.c
        anim_load_file.c
        anim_load_file_interface.c
        anim_process_mcap.c
        anim_process_mcap_interface.c
        anim_reactive.c
        anim_reactive_interface.c
        anim_compute_path.c
        anim_compute_path_interface.c
        anim_optim_path.c
        anim_optim_path_interface.c
        anim_sample_path.c
        anim_sample_path_interface.c
        anim_charac_traj.c
        anim_charac_traj_interface.c
        anim_walk_controller.c
        anim_walk_controller_interface.c
        anim_reac_col.c
        anim_reac_col_interface.c
        anim_show_interface.c
        anim_utils.c
        anim_fft.c)
ENDIF(ANIMATION)
