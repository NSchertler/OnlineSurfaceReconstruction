#include "MeshSplit.h"

int findFirstZero(Eigen::VectorXi v) {
	for (int i = 0; i < v.rows(); i++)
		if (v[i] == 0)
			return i;
	return -1;
}

// void splitMesh(Eigen::MatrixXd V, Eigen::MatrixXi F, std::vector<Eigen::MatrixXd>& subVs, std::vector<Eigen::MatrixXi>& subFs, int bound = 65000) {
// 
// 	// Plot the mesh
// 	//std::vector<igl::opengl::glfw::Viewer> viewers;
// 
// 	// triangle triangle adjacency
// 	Eigen::MatrixXi TT;
// 	igl::triangle_triangle_adjacency(F, TT);
// 
// 	// face discover to record if the face is already into the queue
// 	//int *FD = new int[F.cols()]{ 0 };
// 	Eigen::VectorXi FD = Eigen::VectorXi::Zero(F.rows());
// 
// 	// face queue
// 	std::queue<int> FQ;
// 
// 	// if there is still face not being discovered.
// 	while (!FD.isOnes()) {
// 		// find the first non-one
// 		int curIdx = findFirstZero(FD);
// 		if (curIdx < 0) break;
// 
// 		// visit it
// 		FQ.push(curIdx);
// 
// 		// set of face and vertex for current submesh
// 		std::set<int> FS;
// 		std::set<int> VS;
// 
// 		while (!FQ.empty()) {
// 			int curFace = FQ.front();
// 			FD(curFace) = 1;
// 			FQ.pop();
// 			//std::cout << "\ndealing " << curFace << " pushing neighbours ";
// 
// 			FS.insert(curFace);
// 			for (int i = 0; i < F.cols(); i++)
// 				VS.insert(F(curFace, i));
// 
// 			// get three adjacencies and push into the queue aka visit
// 			for (int i = 0; i < F.cols(); i++) {
// 				int neighbour = TT(curFace, i);
// 
// 				// check if already discoved
// 				if ((neighbour != -1) && (FD(neighbour) == 0)) {
// 					FQ.push(neighbour);
// 					//std::cout << neighbour << "\t";
// 				}
// 				//std::cout << "\n";
// 			}
// 
// 			// check the size of vertices and faces
// 			if (FS.size() > bound || VS.size() > bound)
// 				break;
// 		}
// 
// 		// deal with current submesh, turn set of faces to MatrixXd
// 		Eigen::MatrixXd subV;
// 		Eigen::MatrixXi subF(FS.size(), F.cols());
// 		int i = 0;
// 		for (std::set<int>::iterator it = FS.begin(); it != FS.end(); ++it, i++) {
// 			subF.row(i) = F.row(*it);
// 		}
// 		Eigen::VectorXi UJ;
// 		igl::remove_unreferenced(V, subF, subV, subF, UJ);
// 
// 		subVs.push_back(subV);
// 		subFs.push_back(subF);
// 		// write down
// // 		igl::writeOFF("split" + std::to_string(viewers.size()) + ".off", subV, subF);
// // 		std::cout << "split once\n";
// 
// 		// Plot the mesh
// // 		igl::opengl::glfw::Viewer viewer;
// // 		viewer.data().set_mesh(subV, subF);
// // 		viewers.push_back(viewer);
// 
// 
// 	}
// 
// // 	for each (igl::opengl::glfw::Viewer v in viewers)
// // 	{
// // 		v.launch();
// // 	}
// }

void splitMesh(Eigen::MatrixXd V, Eigen::MatrixXi F, Eigen::MatrixXd C, std::vector<Eigen::MatrixXd>& subVs, std::vector<Eigen::MatrixXi>& subFs, std::vector<Eigen::MatrixXd>& subCs, int bound /*= 64995*/)
{
	// triangle triangle adjacency
	Eigen::MatrixXi TT;
	igl::triangle_triangle_adjacency(F, TT);

	// face discover to record if the face is already into the queue
	//int *FD = new int[F.cols()]{ 0 };
	Eigen::VectorXi FD = Eigen::VectorXi::Zero(F.rows());

	// face queue
	std::queue<int> FQ;

	// if there is still face not being discovered.
	while (!FD.isOnes()) {
		// find the first non-one
		int curIdx = findFirstZero(FD);
		if (curIdx < 0) break;

		// visit it
		FQ.push(curIdx);

		// set of face and vertex for current submesh
		std::set<int> FS;
		std::set<int> VS;

		while (!FQ.empty()) {
			int curFace = FQ.front();
			FD(curFace) = 1;
			FQ.pop();
			//std::cout << "\ndealing " << curFace << " pushing neighbours ";

			FS.insert(curFace);
			for (int i = 0; i < F.cols(); i++)
				VS.insert(F(curFace, i));

			// get three adjacencies and push into the queue aka visit
			for (int i = 0; i < F.cols(); i++) {
				int neighbour = TT(curFace, i);

				// check if already discoved
				if ((neighbour != -1) && (FD(neighbour) == 0)) {
					FQ.push(neighbour);
					FD(neighbour) = 2;
					//std::cout << neighbour << "\t";
				}
				//std::cout << "\n";
			}

			// check the size of vertices and faces
			if (FS.size() > bound || VS.size() > bound)
				break;
		}

		// deal with current submesh, turn set of faces to MatrixXd
		Eigen::MatrixXd subV, subC;
		Eigen::MatrixXi subF(FS.size(), F.cols());
		Eigen::MatrixXi resF(FS.size(), F.cols());
		int i = 0;
		for (std::set<int>::iterator it = FS.begin(); it != FS.end(); ++it, i++) {
			subF.row(i) = F.row(*it);
		}
		Eigen::VectorXi UJ;
		igl::remove_unreferenced(V, subF, subV, resF, UJ);
		igl::remove_unreferenced(C, subF, subC, resF, UJ);

		subVs.push_back(subV);
		subCs.push_back(subC);
		subFs.push_back(resF);

	}
}
