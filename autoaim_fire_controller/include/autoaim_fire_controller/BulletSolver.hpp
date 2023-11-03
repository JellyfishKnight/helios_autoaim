// created by liuhan on 2023/11/4
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
/*
 * ██   ██ ███████ ██      ██  ██████  ███████
 * ██   ██ ██      ██      ██ ██    ██ ██
 * ███████ █████   ██      ██ ██    ██ ███████
 * ██   ██ ██      ██      ██ ██    ██      ██
 * ██   ██ ███████ ███████ ██  ██████  ███████
 */
#pragma once



namespace helios_cv {


typedef struct BulletParams{
    double bullet_speed;
    double bullet_mass;
    double air_coeff;
}BulletParams;


class BulletSolver {
public:
    BulletSolver() = default;

    ~BulletSolver() = default;

    double iterate_pitch();

    void update_bullet_speed();

    void update_bullet_params(const BulletParams& bullet_params);
private:
    BulletParams params_;

    double bullet_speed_;
    double bullet_mass_;


};


} // namespace helios_cv