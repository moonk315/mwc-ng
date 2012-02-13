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

enum enum_param_type_kind {
    PARAM_TYPE_KIND_U8,
    PARAM_TYPE_KIND_U16,
    PARAM_TYPE_KIND_I16,
    PARAM_TYPE_KIND_U32,
    PARAM_TYPE_KIND_I32,
    PARAM_TYPE_KIND_STRUCT,
    PARAM_TYPE_KIND_ARRAY,
};

enum enum_param_type_encoding {
    PARAM_TYPE_ENC_GENERIC,
    PARAM_TYPE_ENC_FP_4x4,
    PARAM_TYPE_ENC_FP_0x10,
    PARAM_TYPE_ENC_FP_1x7,
};

// Scalar types
rtti_type_info_t rtti_u8  = {PARAM_TYPE_KIND_U8,  1,  {0, 0},};  
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
rtti_type_info_t rtti_pid_profile_list = {PARAM_TYPE_KIND_ARRAY, sizeof(pid.setup.profile), {4 , rtti_pid_profile_list_members},};  

// RC Params
rtti_struct_member_t rtti_input_setup_members[] PROGMEM = { 
  {"RATE",   &rtti_u8, PARAM_TYPE_ENC_GENERIC}, 
  {"EXPO",   &rtti_u8, PARAM_TYPE_ENC_GENERIC}, 
};  
rtti_type_info_t rtti_input_setup = {PARAM_TYPE_KIND_STRUCT, sizeof(input.setup), {sizeof(rtti_input_setup_members)/sizeof(rtti_struct_member_t ), rtti_input_setup_members},};  

// Global RTTI info (named type instances)
param_data_t sys_rtti_info[] PROGMEM = {
  {"PID" ,  &rtti_pid_profile_list, pid.setup.profile},
  {"RC" ,   &rtti_input_setup,      &input.setup},
  {"SYSID", &rtti_u8 ,              &mavlink_system.sysid},
};
const uint8_t sys_rtti_info_cnt = sizeof(sys_rtti_info) / sizeof(param_data_t);

uint8_t param_search_first(param_search_rec_t *sr) {
  sr->level = 0;
  sr->idx = 0;
  sr->stack[0].idx = 0;
  memset(sr->name, 0, sizeof(sr->name));
  memcpy_P(&sr->p, &sys_rtti_info[0], sizeof(sr->p));
  sr->inst = sr->p.var;
  sr->stack[0].type = sr->p.type;
  strcpy(sr->name, sr->p.name);
  if (i_scalar(sr->stack[0].type)) return 1;
  return param_search_next(sr);
}  

uint8_t param_search_next(param_search_rec_t *sr) {
  rtti_struct_member_t memb;
  // move pointer in case last member was scalar
  if (i_scalar(sr->stack[sr->level].type)) {
    // TODO: Check code and fix hack
    char *ptr = (char *)sr->inst;
    ptr += sr->stack[sr->level].type->_size;
    sr->inst = ptr;
  }  
  for (;;) {
    // scan members
    if (sr->stack[sr->level].idx < sr->stack[sr->level].type->members.cnt) {
      memcpy_P(&memb, &sr->stack[sr->level].type->members.memb[sr->stack[sr->level].idx], sizeof(rtti_struct_member_t));
      if (sr->stack[sr->level].type->kind != PARAM_TYPE_KIND_ARRAY) strcat(sr->name, "_"); 
      strcat(sr->name, memb.name);
      sr->stack[sr->level].idx++;
      sr->level++;
      sr->stack[sr->level].type = memb.type;
      sr->stack[sr->level].idx = 0;
      if (i_scalar(sr->stack[sr->level].type)) return 1;
    } else {
      if (sr->level == 0) {
        // next parameter
        sr->idx++;
        if (sr->idx >= sys_rtti_info_cnt) return 0;
        memset(sr->name, 0, sizeof(sr->name));
        memcpy_P(&sr->p, &sys_rtti_info[sr->idx], sizeof(sr->p));
        sr->inst = sr->p.var;
        sr->stack[0].type = sr->p.type;
        sr->stack[0].idx = 0;
        strcpy(sr->name, sr->p.name);
        if (i_scalar(sr->stack[0].type)) return 1;
      } else {
        sr->level--;
        // regenerate name from stack
        memset(sr->name, 0, sizeof(sr->name));
        strcpy(sr->name, sr->p.name);
        for (uint8_t i = 0; i < sr->level; i ++) {
          memcpy_P(&memb, &sr->stack[i].type->members.memb[sr->stack[i].idx - 1], sizeof(rtti_struct_member_t));
          if (sr->stack[i].type->kind != PARAM_TYPE_KIND_ARRAY) strcat(sr->name, "_"); 
          strcat(sr->name, memb.name);
        }
      }  
    }
  }  
}  

float scalar_to_float(rtti_type_info_t *t, void *inst) {
  switch (t->kind) {
    case PARAM_TYPE_KIND_U8:  return *(uint8_t *)inst;
    case PARAM_TYPE_KIND_I32: return *(int32_t *)inst;
  }  
  return 0.0f;
}  

inline uint8_t i_scalar(rtti_type_info_t *t) {
  if ((t->kind != PARAM_TYPE_KIND_STRUCT) && (t->kind != PARAM_TYPE_KIND_ARRAY)) 
    return 1;
  else
   return 0;  
}  

void params_expand(char *cont, rtti_type_info_t *t, void *inst) {
  if (i_scalar(t)) dprintf("value: %d \n", round(scalar_to_float(t, inst)));
}  

void params_build_scalar_list() {
  /*
  for (uint8_t i = 0; i < sys_rtti_info_cnt; i++) {
    param_data_t p;
    memcpy_P(&p, &sys_rtti_info[i], sizeof(p));
    dprintf("instance: %s \n", p.name);
    params_expand(p.name, p.type, p.var);
  } */
  param_search_rec_t sr;
  param_search_first(&sr);  
  dprintf("Param: %s = %ld \n", sr.name, (int32_t)round(scalar_to_float(sr.stack[sr.level].type, sr.inst)));
  while (param_search_next(&sr))
    dprintf("Param: %s = %ld \n", sr.name, (int32_t)round(scalar_to_float(sr.stack[sr.level].type, sr.inst)));
}  

inline void Params_Init() {
  param_search_rec_t sr;
  uint8_t cnt = 0;
  if (param_search_next(&sr)) {
    cnt++;
    while (param_search_next(&sr))
      cnt++;
  }  
  sys_param_values_cnt = cnt;
}  


