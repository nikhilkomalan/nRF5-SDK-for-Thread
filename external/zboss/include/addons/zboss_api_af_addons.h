/**
 * Copyright (c) 2018 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef ZBOSS_API_AF_ADDONS_H__
#define ZBOSS_API_AF_ADDONS_H__

#include "zboss_api.h"

/*! \addtogroup zboss_api_af_addons */
/*! @{ */

/**@brief Redefinition of __CAT__ as variadic macro.
 *
 * @param[in]  a   Mandatory argument to concatenate.
 * @param[in]  b   Mandatory argument to concatenate.
 * @param[in]  ... Optional argument to concatenate.
 */
#define __CAT_VA__(a, b, ...)                               a## __VA_ARGS__## b

/**@brief Redefinition of ZB_AF_SIMPLE_DESC_TYPE as variadic macro.
 *
 * @param[in]  in_num   Number of input clusters.
 * @param[in]  out_num  Number of output clusters.
 * @param[in]  ...      Optional argument to concatenate to the type name.
 */
#define ZB_AF_SIMPLE_DESC_TYPE_VA(in_num, out_num, ...)     __CAT_VA__(zb_af_simple_desc_, _t, __VA_ARGS__)

/**@brief Redefinition of ZB_DECLARE_SIMPLE_DESC as variadic macro.
 *
 * @param[in]  in_clusters_count   Number of input clusters.
 * @param[in]  out_clusters_count  Number of output clusters.
 * @param[in]  ...                 Optional argument to concatenate to the type name.
 */
#define ZB_DECLARE_SIMPLE_DESC_VA(in_clusters_count, out_clusters_count, ...)             \
  typedef ZB_PACKED_PRE struct zb_af_simple_desc_## __VA_ARGS__## _s                      \
  {                                                                                       \
    zb_uint8_t    endpoint;                 /* Endpoint */                                \
    zb_uint16_t   app_profile_id;           /* Application profile identifier */          \
    zb_uint16_t   app_device_id;            /* Application device identifier */           \
    zb_bitfield_t app_device_version:4;     /* Application device version */              \
    zb_bitfield_t reserved:4;               /* Reserved */                                \
    zb_uint8_t    app_input_cluster_count;  /* Application input cluster count */         \
    zb_uint8_t    app_output_cluster_count; /* Application output cluster count */        \
    /* Application input and output cluster list */                                       \
    zb_uint16_t   app_cluster_list[(in_clusters_count) + (out_clusters_count)];           \
    zb_uint8_t    cluster_encryption[((in_clusters_count) + (out_clusters_count) + 7)/8]; \
  } ZB_PACKED_STRUCT zb_af_simple_desc_## __VA_ARGS__## _t

/**@brief Redefinition of ZBOSS_DECLARE_DEVICE_CTX_N_EP as variadic macro.
 *
 * @param[in] device_ctx_name  Device context variable name.
 * @param[in] ...              Variables holding context for endpoints.
 */
#define ZBOSS_DECLARE_DEVICE_CTX_EP_VA(device_ctx_name, ...)            \
  zb_af_endpoint_desc_t *ep_list_##device_ctx_name[] = {                \
    __VA_ARGS__                                                         \
  };                                                                    \
  ZBOSS_DECLARE_DEVICE_CTX(device_ctx_name, ep_list_##device_ctx_name,  \
                           (ZB_ZCL_ARRAY_SIZE(ep_list_##device_ctx_name, zb_af_endpoint_desc_t*)))



/** @} */
                           
#endif /* ZBOSS_API_AF_ADDONS_H__ */
