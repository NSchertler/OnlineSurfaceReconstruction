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

#include <boost/asio.hpp>

namespace osr {
	namespace gui {
		namespace loaders
		{
			class DavidTurntableConnection
			{
			public:
				enum Status
				{
					Invalid,
					Stopped,
					Running,
				};

				DavidTurntableConnection();

				~DavidTurntableConnection();

				bool openConnection();

				Status status() const { return _status; }

				double move(double targetDegrees);

			private:

				void handleResponse(const boost::system::error_code& error, std::size_t bytes_transferred);


				boost::asio::io_service io;
				boost::asio::serial_port* turntablePort;

				boost::asio::streambuf response;

				double stepsPerDegree;

				Status _status;
			};
		}
	}
}