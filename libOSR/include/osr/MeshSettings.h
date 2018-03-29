/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Nico Schertler
*/

#pragma once

#include "osr/common.h"
#include "osr/field.h"
#include <memory>

#include <nsessentials/util/Observer.h>

template class OSR_EXPORT std::shared_ptr<osr::IOrientationFieldTraits>;
template class OSR_EXPORT std::shared_ptr<osr::IPositionFieldTraits>;

namespace osr
{
	//Stores all settings for the reconstruction
	class OSR_EXPORT MeshSettings
	{
	public:

		nse::util::Observer<Float> ScaleChanged;
		nse::util::Observer<Float> RegistrationErrorChanged;

		Float scale() const { return _scale; }
		void setScale(Float s)
		{
			if (s != _scale)
			{
				_scale = s;
				ScaleChanged(_scale);
			}
		}

		Float maxRegistrationError() const { return _maxRegistrationError; }
		void setMaxRegistrationError(Float e)
		{
			if (e != _maxRegistrationError)
			{
				_maxRegistrationError = e;
				RegistrationErrorChanged(_maxRegistrationError);
			}
		}

		//Saves the current state of this settings object to file.
		void saveToFile(FILE* f) const
		{
			fwrite(&_scale, sizeof(Float), 1, f);
			fwrite(&_maxRegistrationError, sizeof(float), 1, f);

			int rosyN = rosy->rosy();
			fwrite(&rosyN, sizeof(int), 1, f);

			int posyN = posy->posy();
			fwrite(&posyN, sizeof(int), 1, f);
		}

		//Restores the state of this settings object from file.
		void loadFromFile(FILE* f)
		{
			Float v;
			if(fread(&v, sizeof(Float), 1, f) != 1)
				throw std::runtime_error("Cannot read enough data from file.");
			setScale(v);

			if(fread(&v, sizeof(float), 1, f) != 1)
				throw std::runtime_error("Cannot read enough data from file.");
			setMaxRegistrationError(v);

			int rosyN;
			if(fread(&rosyN, sizeof(int), 1, f) != 1)
				throw std::runtime_error("Cannot read enough data from file.");
			rosy = std::shared_ptr<IOrientationFieldTraits>(getOrientationFieldTraits(rosyN));

			int posyN;
			if(fread(&posyN, sizeof(int), 1, f) != 1)
				throw std::runtime_error("Cannot read enough data from file.");
			posy = std::shared_ptr<IPositionFieldTraits>(getPositionFieldTraits(posyN));
		}

	private:
		Float _scale;

		float _maxRegistrationError;

	public:
		std::shared_ptr<IOrientationFieldTraits> rosy;
		std::shared_ptr<IPositionFieldTraits> posy;

		//random subsampling of scans; this value represents a percentage in (0, 1]
		double scanSubsample = 1.0;
		float smoothness = 0.8f;

		bool scanCutoff = false;
		float scanCutoffHeight = 30.0f; //y-axis
	};
}