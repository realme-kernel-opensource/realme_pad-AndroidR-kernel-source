#ifndef _LC_AUDIO_INFO_H
#define _LC_AUDIO_INFO_H

enum smartpa_type {
	INVALID = 0,
	FS1894SU,
	AW88194A,
	AW88394,
	SMARTPA_TYPE_MAX
};

enum smartpa_num {
	SMARTPA_NUM_NONE = 0,
	SMARTPA_NUM_ONE,
	SMARTPA_NUM_TWO,
	SMARTPA_NUM_THREE,
	SMARTPA_NUM_FOUR,
	SMARTPA_NUM_FIVE,
	SMARTPA_NUM_SIX,
	SMARTPA_NUM_SEVEN,
	SMARTPA_NUM_EIGHT,
	SMARTPA_NUM_MAX
};

#define SMARTPA_TYPE_BUFF_SIZE 32
#define SMARTPA_NUM_BUFF_SIZE 32

struct lc_audio_info {
	char smartpa_type_str[SMARTPA_TYPE_BUFF_SIZE];
	char smartpa_num_str[SMARTPA_NUM_BUFF_SIZE];
};

	void set_smartpa_type(const char *buf, int len);
	enum smartpa_type get_smartpa_type(void);
	int get_smartpa_num(void);

#endif