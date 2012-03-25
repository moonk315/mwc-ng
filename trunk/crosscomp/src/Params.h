/**
 * MultiWii NG 0.1 - 2012
 * RTTI for main controller parameters
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

// Scalar types
rtti_type_info_t rtti_u8  = {PARAM_TYPE_KIND_U8,  1,  {0, 0},};
rtti_type_info_t rtti_i16 = {PARAM_TYPE_KIND_I16, 2,  {0, 0},};
rtti_type_info_t rtti_u16 = {PARAM_TYPE_KIND_U16, 2,  {0, 0},};
rtti_type_info_t rtti_i32 = {PARAM_TYPE_KIND_I32, 4,  {0, 0},};

// PID Profiles
rtti_struct_member_t rtti_pid_terms_members[] PROGMEM = {
  {"P",    &rtti_u8,  PARAM_TYPE_ENC_FP_4x4},
  {"I",    &rtti_u8,  PARAM_TYPE_ENC_FP_0x10},
  {"D",    &rtti_u8,  PARAM_TYPE_ENC_GENERIC},
  {"FF",   &rtti_u8,  PARAM_TYPE_ENC_FP_1x7},
  {"WND",  &rtti_i32, PARAM_TYPE_ENC_GENERIC},
};
rtti_type_info_t rtti_pid_terms = {PARAM_TYPE_KIND_STRUCT, sizeof(pid_terms_t), {sizeof(rtti_pid_terms_members)/sizeof(rtti_struct_member_t), rtti_pid_terms_members},};

rtti_struct_member_t rtti_pid_channels_members[] PROGMEM = {
  {"R",    &rtti_pid_terms,  PARAM_TYPE_ENC_GENERIC},
  {"P",    &rtti_pid_terms,  PARAM_TYPE_ENC_GENERIC},
  {"Y",    &rtti_pid_terms,  PARAM_TYPE_ENC_GENERIC},
  {"T",    &rtti_pid_terms,  PARAM_TYPE_ENC_GENERIC},
};
rtti_type_info_t rtti_pid_channels = {PARAM_TYPE_KIND_STRUCT, sizeof(pid_channels_t), {sizeof(rtti_pid_channels_members)/sizeof(rtti_struct_member_t), rtti_pid_channels_members},};

rtti_struct_member_t rtti_pid_profile_members[] PROGMEM = {
  {"I",   &rtti_pid_channels, PARAM_TYPE_ENC_GENERIC},
  {"O",   &rtti_pid_channels, PARAM_TYPE_ENC_GENERIC},
};
rtti_type_info_t rtti_pid_profile = {PARAM_TYPE_KIND_STRUCT, sizeof(pid_profile_t), {sizeof(rtti_pid_profile_members)/sizeof(rtti_struct_member_t), rtti_pid_profile_members},};

rtti_struct_member_t rtti_pid_profile_list_members[] PROGMEM = {
  {"0",   &rtti_pid_profile, PARAM_TYPE_ENC_GENERIC},
  {"1",   &rtti_pid_profile, PARAM_TYPE_ENC_GENERIC},
  {"2",   &rtti_pid_profile, PARAM_TYPE_ENC_GENERIC},
  {"3",   &rtti_pid_profile, PARAM_TYPE_ENC_GENERIC},
};
rtti_type_info_t rtti_pid_profile_list = {PARAM_TYPE_KIND_ARRAY, /*sizeof(pid.setup.profile)*/0, {4 , rtti_pid_profile_list_members},};

// RC Params
rtti_struct_member_t rtti_rc_map_list_members[] PROGMEM = {
  {"0",   &rtti_u8, PARAM_TYPE_ENC_GENERIC},
  {"1",   &rtti_u8, PARAM_TYPE_ENC_GENERIC},
  {"2",   &rtti_u8, PARAM_TYPE_ENC_GENERIC},
  {"3",   &rtti_u8, PARAM_TYPE_ENC_GENERIC},
};
rtti_type_info_t rtti_map_list = {PARAM_TYPE_KIND_ARRAY, 0, {4 , rtti_rc_map_list_members},};

rtti_struct_member_t rtti_input_setup_members[] PROGMEM = {
  {"RATE",   &rtti_u8, PARAM_TYPE_ENC_RCR},
  {"EXPO",   &rtti_u8, PARAM_TYPE_ENC_GENERIC},
  {"PSW",    &rtti_u8, PARAM_TYPE_ENC_GENERIC},
  {"MSW",    &rtti_u8, PARAM_TYPE_ENC_GENERIC},
  {"PMAP",   &rtti_map_list, PARAM_TYPE_ENC_GENERIC},
  {"MMAP",   &rtti_map_list, PARAM_TYPE_ENC_GENERIC},
};
rtti_type_info_t rtti_input_setup = {PARAM_TYPE_KIND_STRUCT, sizeof(input.setup), {sizeof(rtti_input_setup_members)/sizeof(rtti_struct_member_t ), rtti_input_setup_members},};

// Voltage monitor
rtti_struct_member_t rtti_vbat_setup_members[] PROGMEM = {
  {"SCAL",   &rtti_u8,  PARAM_TYPE_ENC_GENERIC},
  {"W1",  &rtti_i16, PARAM_TYPE_ENC_FPD_1000},
  {"W2",  &rtti_i16, PARAM_TYPE_ENC_FPD_1000},
};
rtti_type_info_t rtti_vbat_setup = {PARAM_TYPE_KIND_STRUCT, sizeof(flight.setup.vbat), {sizeof(rtti_vbat_setup_members)/sizeof(rtti_struct_member_t ), rtti_vbat_setup_members},};

// Global RTTI info (named type instances)
param_data_t sys_rtti_info[] PROGMEM = {
  {"PID" ,  &rtti_pid_profile_list, pid.setup.profile},
  {"RC" ,   &rtti_input_setup,      &input.setup},
  {"VBAT",  &rtti_vbat_setup,       &flight.setup.vbat},
  {"SYSID", &rtti_u8 ,              &mavlink_system.sysid},
};
const uint8_t sys_rtti_info_cnt = sizeof(sys_rtti_info) / sizeof(param_data_t);

inline uint8_t param_get_encoding(param_search_rec_t *sr) {
  return sr->stack[sr->level].encoding;
}

inline rtti_type_info_t *param_get_type(param_search_rec_t *sr) {
  return sr->stack[sr->level].type;
}

inline void *param_get_inst(param_search_rec_t *sr) {
  return sr->inst;
}

inline uint8_t param_get_type_kind(param_search_rec_t *sr) {
  return param_get_type(sr)->kind;
}

inline uint8_t param_is_float(param_search_rec_t *sr) {
  return (param_get_encoding(sr) != PARAM_TYPE_ENC_GENERIC);
}

uint8_t is_scalar(rtti_type_info_t *t) {
  return ((t->kind != PARAM_TYPE_KIND_STRUCT) && (t->kind != PARAM_TYPE_KIND_ARRAY));
}

float scalar_to_float(rtti_type_info_t *t, uint8_t encoding, void *inst) {
  float res;
  switch (t->kind) {
    case PARAM_TYPE_KIND_U8:  res = *(uint8_t *)inst; break;
    case PARAM_TYPE_KIND_I16: res = *(int16_t *)inst; break;
    case PARAM_TYPE_KIND_I32: res = *(int32_t *)inst; break;
    default: res = 0.0f;
  }
  switch (encoding) {
    case PARAM_TYPE_ENC_FP_4x4:  res /= 16.0f;   break;
    case PARAM_TYPE_ENC_FP_0x10: res /= 1024.0f; break;
    case PARAM_TYPE_ENC_FP_1x7:  res /= 128.0f;  break;
    case PARAM_TYPE_ENC_RCR:     res /= 0.5f;    break;
    case PARAM_TYPE_ENC_FPD_1000:res /= 1000.0f; break;
  }
 return res;
}

void float_to_scalar(rtti_type_info_t *t, float val, uint8_t encoding, void *inst) {
  float tmp = 1.0f;
  switch (encoding) {
    case PARAM_TYPE_ENC_FP_4x4:  tmp = 16.0f; break;
    case PARAM_TYPE_ENC_FP_0x10: tmp = 1024.0f; break;
    case PARAM_TYPE_ENC_FP_1x7:  tmp = 128.0f; break;
    case PARAM_TYPE_ENC_RCR:     tmp = 0.5f; break;
    case PARAM_TYPE_ENC_FPD_1000:tmp = 1000.0f; break;
  }
  val = round(val * tmp);
  switch (t->kind) {
    case PARAM_TYPE_KIND_U8:  *(uint8_t *)inst = val; break;
    case PARAM_TYPE_KIND_I16: *(int16_t *)inst = val; break;
    case PARAM_TYPE_KIND_I32: *(int32_t *)inst = val; break;
  }
}

inline float param_get_val(param_search_rec_t *sr) {
  return scalar_to_float(param_get_type(sr), param_get_encoding(sr), param_get_inst(sr));
}

inline void param_set_val(param_search_rec_t *sr, float val) {
  float_to_scalar(param_get_type(sr), val, param_get_encoding(sr), param_get_inst(sr));
}

uint8_t param_search_next(param_search_rec_t *sr);

uint8_t param_search_first(param_search_rec_t *sr) {
  sr->level = 0;
  sr->idx = 0;
  sr->stack[0].idx = 0;
  memset(sr->name, 0, sizeof(sr->name));
  memcpy_P(&sr->p, &sys_rtti_info[0], sizeof(sr->p));
  sr->inst = sr->p.var;
  sr->stack[0].type = sr->p.type;
  sr->stack[0].encoding = PARAM_TYPE_ENC_GENERIC;
  strcpy(sr->name, sr->p.name);
  if (is_scalar(sr->stack[0].type)) return 1;
  return param_search_next(sr);
}

uint8_t param_search_next(param_search_rec_t *sr) {
  rtti_struct_member_t memb;
  struct struct_node_search_rec *st;
  // move pointer in case last member was scalar
  if (is_scalar(sr->stack[sr->level].type)) {
    char *ptr = (char *)sr->inst;
    ptr += sr->stack[sr->level].type->_size;
    sr->inst = ptr;
  }
  for (;;) {
    // scan members
    st = &sr->stack[sr->level];
    if (st->idx < st->type->members.cnt) {
      memcpy_P(&memb, &st->type->members.memb[st->idx], sizeof(rtti_struct_member_t));
      if (st->type->kind != PARAM_TYPE_KIND_ARRAY) strcat_P(sr->name, PSTR("_"));
      strcat(sr->name, memb.name);
      sr->stack[sr->level].idx++;
      sr->level++;
      st = &sr->stack[sr->level];
      st->type = memb.type;
      st->idx = 0;
      st->encoding = memb.encoding;
      if (is_scalar(st->type)) return 1;
    } else {
      if (sr->level == 0) {
        // next parameter
        sr->idx++;
        if (sr->idx >= sys_rtti_info_cnt) return 0;
        memset(sr->name, 0, sizeof(sr->name));
        memcpy_P(&sr->p, &sys_rtti_info[sr->idx], sizeof(sr->p));
        sr->inst = sr->p.var;
        st = &sr->stack[0];
        st->type = sr->p.type;
        st->idx = 0;
        st->encoding = PARAM_TYPE_ENC_GENERIC;
        strcpy(sr->name, sr->p.name);
        if (is_scalar(st->type)) return 1;
      } else {
        sr->level--;
        // regenerate name from stack
        memset(sr->name, 0, sizeof(sr->name));
        strcpy(sr->name, sr->p.name);
        for (uint8_t i = 0; i < sr->level; i ++) {
          st = &sr->stack[i];
          memcpy_P(&memb, &st->type->members.memb[st->idx - 1], sizeof(rtti_struct_member_t));
          if (st->type->kind != PARAM_TYPE_KIND_ARRAY) strcat_P(sr->name, PSTR("_"));
          strcat(sr->name, memb.name);
        }
      }
    }
  }
}

uint8_t param_search_by_name(param_search_rec_t *sr, char *name) {
  if (param_search_first(sr)) {
    if (strcmp(sr->name, name) == 0) return 1;
    while (param_search_next(sr)) {
      if (strcmp(sr->name, name) == 0) return 1;
    }
  }
  return 0;
}

uint8_t param_search_by_idx(param_search_rec_t *sr, uint8_t idx) {
  uint8_t cnt = 0;
  if (param_search_first(sr)) {
    if (cnt == idx) return 1;
    cnt++;
    while (param_search_next(sr)) {
      if (cnt == idx) return 1;
      cnt++;
    }
  }
  return 0;
}


inline void Params_Init() {
  param_search_rec_t sr;
  uint8_t cnt = 0;
  if (param_search_first(&sr)) {
    cnt++;
    while (param_search_next(&sr))
      cnt++;
  }
  sys_param_values_cnt = cnt;
}


