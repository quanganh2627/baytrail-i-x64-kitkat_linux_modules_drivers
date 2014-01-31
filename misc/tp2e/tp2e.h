#ifndef _TP2E_H_
#define _TP2E_H_

#include <linux/kct.h>

#undef kct_log
#define kct_log(ev_type, submitter_name, ev_name, data0, data1, data2) \
	do {								\
		if (kct_alloc_event) {					\
			struct ct_event *__ev =			\
				kct_alloc_event(submitter_name,	\
						ev_name,		\
						ev_type,		\
						GFP_ATOMIC,		\
						0);			\
			if (__ev) {					\
				if (data0)				\
					kct_add_attchmt(&__ev,		\
							CT_ATTCHMT_DATA0, \
							strlen(data0) + 1, \
							data0, GFP_ATOMIC); \
				if (data1)				\
					kct_add_attchmt(&__ev,		\
							CT_ATTCHMT_DATA1, \
							strlen(data1) + 1, \
							data1, GFP_ATOMIC); \
				if (data2)				\
					kct_add_attchmt(&__ev,		\
							CT_ATTCHMT_DATA2, \
							strlen(data2) + 1, \
							data2, GFP_ATOMIC); \
				kct_log_event(__ev, GFP_ATOMIC);	\
			}						\
		}							\
	} while (0)


#define DEFINE_PROBE(event, probe)

#endif /* _TP2E_H_ */

#ifdef DECLARE_TP2E_ELT
#undef DEFINE_PROBE
#define DEFINE_PROBE(event, probe)				\
	static struct tp2e_element tp2e_##event = {			\
		.system = __stringify(TRACE_SYSTEM),			\
		.name = __stringify(event),				\
		.probe_fn = (void *)probe,				\
	};
#endif /* DECLARE_TP2E_ELT */

#ifdef ADD_TP2E_ELT
#undef DEFINE_PROBE
#define DEFINE_PROBE(event, probe)				\
	do {								\
		INIT_LIST_HEAD(&tp2e_##event.list);			\
		list_add_tail(&tp2e_##event.list, &tp2e_list);		\
	} while (0)
#endif /* ADD_TP2E_ELT */

