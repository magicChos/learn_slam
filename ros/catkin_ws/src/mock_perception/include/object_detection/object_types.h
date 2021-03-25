#pragma once
#include <string>

enum class ObjectTypes
{
	obj_background,
	obj_bottle,
	obj_shoes,
	obj_wire,
	TypesCount
};

enum class CocoTypes
{
	coco_obj_person,
	coco_obj_bicycle,
	coco_obj_car,
	coco_obj_motorbike,
	coco_obj_aeroplane,
	coco_obj_bus,
	coco_obj_train,
	coco_obj_truck,
	coco_obj_boat,
	coco_obj_traffic_light,
	coco_obj_fire_hydrant,
	coco_obj_stop_sign,
	coco_obj_parking_meter,
	coco_obj_bench,
	coco_obj_bird,
	coco_obj_cat,
	coco_obj_dog,
	coco_obj_horse,
	coco_obj_sheep,
	coco_obj_cow,
	coco_obj_elephant,
	coco_obj_bear,
	coco_obj_zebra,
	coco_obj_giraffe,
	coco_obj_backpack,
	coco_obj_umbrella,
	coco_obj_handbag,
	coco_obj_tie,
	coco_obj_suitcase,
	coco_obj_frisbee,
	coco_obj_skis,
	coco_obj_snowboard,
	coco_obj_sports_ball,
	coco_obj_kite,
	coco_obj_baseball_bat,
	coco_obj_baseball_glove,
	coco_obj_skateboard,
	coco_obj_surfboard,
	coco_obj_tennis_racket,
	coco_obj_bottle,
	coco_obj_wine_glass,
	coco_obj_cup,
	coco_obj_fork,
	coco_obj_knife,
	coco_obj_spoon,
	coco_obj_bowl,
	coco_obj_banana,
	coco_obj_apple,
	coco_obj_sandwich,
	coco_obj_orange,
	coco_obj_broccoli,
	coco_obj_carrot,
	coco_obj_hot_dog,
	coco_obj_pizza,
	coco_obj_donut,
	coco_obj_cake,
	coco_obj_chair,
	coco_obj_sofa,
	coco_obj_pottedplant,
	coco_obj_bed,
	coco_obj_diningtable,
	coco_obj_toilet,
	coco_obj_tvmonitor,
	coco_obj_laptop,
	coco_obj_mouse,
	coco_obj_remote,
	coco_obj_keyboard,
	coco_obj_cell_phone,
	coco_obj_microwave,
	coco_obj_oven,
	coco_obj_toaster,
	coco_obj_sink,
	coco_obj_refrigerator,
	coco_obj_book,
	coco_obj_clock,
	coco_obj_vase,
	coco_obj_scissors,
	coco_obj_teddy_bear,
	coco_obj_hair_drier,
	coco_obj_toothbrush
};

typedef int objtype_t;
constexpr objtype_t bad_type = -1;

class TypeConverter
{
public:
	static std::string Type2Str(objtype_t type)
	{
		return (type == bad_type) ? m_badTypeName : m_typeNames[type];
	}

	static objtype_t Str2Type(const std::string& str)
	{
		for (size_t i = 0; i < m_typeNames->size(); ++i)
		{
			if (str == m_typeNames[i])
				return static_cast<objtype_t>(i);
		}
		return bad_type;
	}

private:
	static std::string m_typeNames[(size_t)ObjectTypes::TypesCount];
	static std::string m_badTypeName;
};
