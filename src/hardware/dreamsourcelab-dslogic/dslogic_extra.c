#include "dslogic_extra.h" 

SR_PRIV uint16_t ds_trigger_get_mask0(uint16_t stage, uint16_t msc, uint16_t lsc, gboolean qutr_mode, gboolean half_mode)
{
    assert(stage <= TriggerStages);
    assert(lsc <= msc);
    assert(msc < MaxTriggerProbes);

    uint16_t mask = 0;
    const uint16_t qutr_mask = (0xffff >> (TriggerProbes - TriggerProbes/4));
    const uint16_t half_mask = (0xffff >> (TriggerProbes - TriggerProbes/2));
    int i;

    for (i = msc; i >= lsc ; i--) {
        mask = (mask << 1);
        mask += ((trigger->trigger0[stage][i] == 'X') | (trigger->trigger0[stage][i] == 'C'));
    }

    if (qutr_mode)
        mask = ((mask & qutr_mask) << (TriggerProbes/4*3)) +
               ((mask & qutr_mask) << (TriggerProbes/4*2)) +
               ((mask & qutr_mask) << (TriggerProbes/4*1)) +
               ((mask & qutr_mask) << (TriggerProbes/4*0));
    else if (half_mode)
        mask = ((mask & half_mask) << (TriggerProbes/2*1)) +
               ((mask & half_mask) << (TriggerProbes/2*0));

    return mask;
}
SR_PRIV uint16_t ds_trigger_get_mask1(uint16_t stage, uint16_t msc, uint16_t lsc, gboolean qutr_mode, gboolean half_mode)
{
    assert(stage <= TriggerStages);
    assert(lsc <= msc);
    assert(msc < MaxTriggerProbes);

    uint16_t mask = 0;
    const uint16_t qutr_mask = (0xffff >> (TriggerProbes - TriggerProbes/4));
    const uint16_t half_mask = (0xffff >> (TriggerProbes - TriggerProbes/2));
    int i;

    for (i = msc; i >= lsc ; i--) {
        mask = (mask << 1);
        mask += ((trigger->trigger1[stage][i] == 'X') | (trigger->trigger1[stage][i] == 'C'));
    }

    if (qutr_mode)
        mask = ((mask & qutr_mask) << (TriggerProbes/4*3)) +
               ((mask & qutr_mask) << (TriggerProbes/4*2)) +
               ((mask & qutr_mask) << (TriggerProbes/4*1)) +
               ((mask & qutr_mask) << (TriggerProbes/4*0));
    else if (half_mode)
        mask = ((mask & half_mask) << (TriggerProbes/2*1)) +
               ((mask & half_mask) << (TriggerProbes/2*0));

    return mask;
}
SR_PRIV uint16_t ds_trigger_get_value0(uint16_t stage, uint16_t msc, uint16_t lsc, gboolean qutr_mode, gboolean half_mode)
{
    assert(stage <= TriggerStages);
    assert(lsc <= msc);
    assert(msc < MaxTriggerProbes);

    uint16_t value = 0;
    const uint16_t qutr_mask = (0xffff >> (TriggerProbes - TriggerProbes/4));
    const uint16_t half_mask = (0xffff >> (TriggerProbes - TriggerProbes/2));
    int i;

    for (i = msc; i >= lsc ; i--) {
        value = (value << 1);
        value += ((trigger->trigger0[stage][i] == '1') | (trigger->trigger0[stage][i] == 'R'));
    }

    if (qutr_mode)
        value = ((value & qutr_mask) << (TriggerProbes/4*3)) +
                ((value & qutr_mask) << (TriggerProbes/4*2)) +
                ((value & qutr_mask) << (TriggerProbes/4*1)) +
                ((value & qutr_mask) << (TriggerProbes/4*0));
    else if (half_mode)
        value = ((value & half_mask) << (TriggerProbes/2*1)) +
                ((value & half_mask) << (TriggerProbes/2*0));

    return value;
}
SR_PRIV uint16_t ds_trigger_get_value1(uint16_t stage, uint16_t msc, uint16_t lsc, gboolean qutr_mode, gboolean half_mode)
{
    assert(stage <= TriggerStages);
    assert(lsc <= msc);
    assert(msc < MaxTriggerProbes);

    uint16_t value = 0;
    const uint16_t qutr_mask = (0xffff >> (TriggerProbes - TriggerProbes/4));
    const uint16_t half_mask = (0xffff >> (TriggerProbes - TriggerProbes/2));
    int i;

    for (i = msc; i >= lsc ; i--) {
        value = (value << 1);
        value += ((trigger->trigger1[stage][i] == '1') | (trigger->trigger1[stage][i] == 'R'));
    }

    if (qutr_mode)
        value = ((value & qutr_mask) << (TriggerProbes/4*3)) +
                ((value & qutr_mask) << (TriggerProbes/4*2)) +
                ((value & qutr_mask) << (TriggerProbes/4*1)) +
                ((value & qutr_mask) << (TriggerProbes/4*0));
    else if (half_mode)
        value = ((value & half_mask) << (TriggerProbes/2*1)) +
                ((value & half_mask) << (TriggerProbes/2*0));

    return value;
}
SR_PRIV uint16_t ds_trigger_get_edge0(uint16_t stage, uint16_t msc, uint16_t lsc, gboolean qutr_mode, gboolean half_mode)
{
    assert(stage <= TriggerStages);
    assert(lsc <= msc);
    assert(msc < MaxTriggerProbes);

    uint16_t edge = 0;
    const uint16_t qutr_mask = (0xffff >> (TriggerProbes - TriggerProbes/4));
    const uint16_t half_mask = (0xffff >> (TriggerProbes - TriggerProbes/2));
    int i;

    for (i = msc; i >= lsc ; i--) {
        edge = (edge << 1);
        edge += ((trigger->trigger0[stage][i] == 'R') | (trigger->trigger0[stage][i] == 'F') |
                 (trigger->trigger0[stage][i] == 'C'));
    }

    if (qutr_mode)
        edge = ((edge & qutr_mask) << (TriggerProbes/4*3)) +
               ((edge & qutr_mask) << (TriggerProbes/4*2)) +
               ((edge & qutr_mask) << (TriggerProbes/4*1)) +
               ((edge & qutr_mask) << (TriggerProbes/4*0));
    else if (half_mode)
        edge = ((edge & half_mask) << (TriggerProbes/2*1)) +
               ((edge & half_mask) << (TriggerProbes/2*0));

    return edge;
}
SR_PRIV uint16_t ds_trigger_get_edge1(uint16_t stage, uint16_t msc, uint16_t lsc, gboolean qutr_mode, gboolean half_mode)
{
    assert(stage <= TriggerStages);
    assert(lsc <= msc);
    assert(msc < MaxTriggerProbes);

    uint16_t edge = 0;
    const uint16_t qutr_mask = (0xffff >> (TriggerProbes - TriggerProbes/4));
    const uint16_t half_mask = (0xffff >> (TriggerProbes - TriggerProbes/2));
    int i;

    for (i = msc; i >= lsc ; i--) {
        edge = (edge << 1);
        edge += ((trigger->trigger1[stage][i] == 'R') | (trigger->trigger1[stage][i] == 'F') |
                 (trigger->trigger1[stage][i] == 'C'));
    }

    if (qutr_mode)
        edge = ((edge & qutr_mask) << (TriggerProbes/4*3)) +
               ((edge & qutr_mask) << (TriggerProbes/4*2)) +
               ((edge & qutr_mask) << (TriggerProbes/4*1)) +
               ((edge & qutr_mask) << (TriggerProbes/4*0));
    else if (half_mode)
        edge = ((edge & half_mask) << (TriggerProbes/2*1)) +
               ((edge & half_mask) << (TriggerProbes/2*0));

    return edge;
}
