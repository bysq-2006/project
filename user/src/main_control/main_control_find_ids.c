#include "main_control_find_ids.h"

uint8 main_control_find_ids_main(main_control_context_t *ctx,
                                 const openart_pose_t *pose,
                                 const openart_map_t *map)
{
    if((0 == ctx) || (0 == pose) || (0 == map))
    {
        return 0;
    }

    return 1;
}
