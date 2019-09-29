#include "../openhmdi.h"
#include "../omath.h"

typedef struct kalman_pose_s kalman_pose;

kalman_pose *kalman_pose_new(ohmd_context* ohmd_ctx);
void kalman_pose_update(kalman_pose *kp, double time, const vec3f* pos, quatf *orient);
void kalman_pose_get_estimated(kalman_pose *kp, vec3f* pos, quatf *orient);
void kalman_pose_free (kalman_pose *kp);

