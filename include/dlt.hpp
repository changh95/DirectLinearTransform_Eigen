#include "common.hpp"

#include <limits>
#include <random>

class DLT
{
public:
    using MatrixB = Eigen::MatrixXd; // Matrix B from DLT equation : Bp = 0;
    using MatrixP = Eigen::MatrixXd;
    using Pts2D = std::vector<Vec2D>;
    using Pts3D = std::vector<Vec3D>;

    struct Solution
    {
        MatrixP mat = MatrixP::Zero(3, 4);
        std::vector<double> reprojErr = std::vector<double>(30, std::numeric_limits<double>::max());
        double avgReprojError = std::numeric_limits<double>::max();
        int numInliers = std::numeric_limits<int>::min();

        Solution() = default;
    };

    struct Config
    {
        int max_iter_RANSAC = 1000;
        double reprojErrorThreshold = 3.0;
        int min_Inliers = 10;
        double minAvgReprojError = 5.0;
        int max_repeated_output = 200;
    };

    explicit DLT(const Pts2D& imgCoords, const Pts3D& posMarkers) : mImgCoords(imgCoords), mPosMarkers(posMarkers){};

    Solution run()
    {
        assert(mImgCoords.size() == mPosMarkers.size());

        mMatrixProj_OD = doOverDeterminedDLT();
        std::cout << "Over-determined DLT gave average reprojection error of " << mMatrixProj_OD.avgReprojError << std::endl;

        mMatrixProj_RANSAC = doRANSACDLT();
        std::cout << "Minimal DLT solver + RANSAC gave average reprojection error of " << mMatrixProj_RANSAC.avgReprojError << std::endl;

        if (mMatrixProj_RANSAC.avgReprojError < mMatrixProj_OD.avgReprojError)
        {
            std::cout << std::endl << "Since minimal DLT solver + RANSAC has a lower average reprojection error, We will use DLT+RANSAC result as the solution" << std::endl;
            mMatrixC = mMatrixProj_RANSAC;
        }
        else
        {
            std::cout << std::endl << "Since Over-determined DLT has a lower average reprojection error, We will use Over-determined DLT result as the solution" << std::endl;
            mMatrixC = mMatrixProj_OD;
        }

        return mMatrixC;
    };

    Config& getConfig() { return mConfig; }

private:
    Config mConfig;
    const Pts2D& mImgCoords;
    const Pts3D& mPosMarkers;
    Solution mMatrixProj_OD;
    Solution mMatrixProj_RANSAC;
    Solution mMatrixC;

    Mat4x4D QRdecomp(const Solution& sol)
    {
        // Code implementation incomplete

        // P = [H|h]
        Mat3x3D H_mat;
        Vec3D h_mat;
        H_mat.block<3, 3>(0, 0) = sol.mat.block<3, 3>(0, 0);
        h_mat.block<3, 1>(0, 0) = sol.mat.block<3, 1>(0, 3);

        // t = -H^-1 * h;
        Vec3D t;
        t = -H_mat.inverse() * h_mat;

        // QR(H^-1) = R^T * K^-1
        Eigen::ColPivHouseholderQR<Mat3x3D> qr(H_mat);

        // R_z_pi = [-1, 0, 0 ; 0, -1, 0; 0, 0, 1], to
        Mat3x3D R_z_pi_mat = Mat3x3D::Identity();
        R_z_pi_mat(0, 0) = -1.0;
        R_z_pi_mat(1, 1) = -1.0;

        Mat3x3D Rot_mat;
        Rot_mat = qr.matrixQ().transpose();
        Rot_mat = R_z_pi_mat * Rot_mat;

        Mat3x3D K_mat;
        K_mat = qr.matrixR();
        K_mat = K_mat.inverse();
        K_mat = K_mat * R_z_pi_mat;
        K_mat = K_mat / K_mat(2, 2);

        std::cout << "projMat is " << std::endl << sol.mat << std::endl;
        std::cout << "XO is " << std::endl << t << std::endl;
        std::cout << "Rot_mat is " << std::endl << Rot_mat << std::endl;
        std::cout << "K_mat is " << std::endl << K_mat << std::endl;

        std::cout << "K_mat * Rot_mat = " << std::endl << K_mat * Rot_mat << std::endl;

        Mat4x4D temp; // Code implementation incomplete
        return temp;
    };

    Solution doRANSACDLT()
    {
        MatrixB mat = MatrixB::Random(12, 12);
        mMatrixProj_RANSAC.mat = MatrixP::Zero(3, 4);
        Solution bestModel;

        static std::random_device rd;
        static std::mt19937 rng{ rd() };
        static std::uniform_int_distribution<int> uid(0, 29);

        int iter_RANSAC = 0;
        int repeated_output = 0;

        std::vector<int> indices;
        indices.reserve(6);

        while (iter_RANSAC < mConfig.max_iter_RANSAC)
        {
            // Early termination
            if (bestModel.numInliers > mConfig.min_Inliers && bestModel.avgReprojError < mConfig.minAvgReprojError)
                break;

            // Terminate if output remains the same for a long time
            if (repeated_output > mConfig.max_repeated_output)
                break;

            // Sample points
            indices.clear();
            indices.push_back(uid(rng));
            while (indices.size() < 6)
            {
                indices.push_back(uid(rng));
                indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
            }

            int row = 0;
            for (const auto idx : indices)
            {
                const double X = mPosMarkers[idx][0];
                const double Y = mPosMarkers[idx][1];
                const double Z = mPosMarkers[idx][2];
                const double u = mImgCoords[idx][0];
                const double v = mImgCoords[idx][1];

                // 0,0,0,0 -X, -Y, -Z, -1, vX, vY, vZ, v
                mat(row, 0) = -X;
                mat(row, 1) = -Y;
                mat(row, 2) = -Z;
                mat(row, 3) = -1.0;
                mat(row, 4) = 0.0;
                mat(row, 5) = 0.0;
                mat(row, 6) = 0.0;
                mat(row, 7) = 0.0;
                mat(row, 8) = u * X;
                mat(row, 9) = u * Y;
                mat(row, 10) = u * Z;
                mat(row, 11) = u;

                row++;

                // -X -Y -Z, -1, 0,0,0,0, uX, uY, uZ ,u
                mat(row, 0) = 0.0;
                mat(row, 1) = 0.0;
                mat(row, 2) = 0.0;
                mat(row, 3) = 0.0;
                mat(row, 4) = -X;
                mat(row, 5) = -Y;
                mat(row, 6) = -Z;
                mat(row, 7) = -1.0;
                mat(row, 8) = v * X;
                mat(row, 9) = v * Y;
                mat(row, 10) = v * Z;
                mat(row, 11) = v;

                row++;
            }

            Eigen::BDCSVD<MatrixB> svd(mat, Eigen::ComputeThinU | Eigen::ComputeThinV);

            // A rare failure case due to rank deficiency
            if (svd.rank() != 12)
                continue;

            for (int row = 0; row < 3; row++)
                for (int col = 0; col < 4; col++)
                    mMatrixProj_RANSAC.mat(row, col) = svd.matrixV().col(11)[4 * row + col];

            evaluate_RANSAC(mMatrixProj_RANSAC, mConfig.reprojErrorThreshold);

            if (mMatrixProj_RANSAC.numInliers > bestModel.numInliers)
            {
                bestModel = mMatrixProj_RANSAC;
                repeated_output = 0;
            }
            else
                repeated_output++;
        }
        return bestModel;
    };

    Solution doOverDeterminedDLT()
    {
        MatrixB mat = MatrixB::Random(60, 12);
        mMatrixProj_OD.mat = MatrixP::Zero(3, 4);
        int ptIdx = 0;

        for (int row = 0; row < mat.rows(); row++)
        {
            if (row & 1)
            {
                // 0,0,0,0 -X, -Y, -Z, -1, vX, vY, vZ, v
                double X = mPosMarkers[ptIdx][0];
                double Y = mPosMarkers[ptIdx][1];
                double Z = mPosMarkers[ptIdx][2];
                double u = mImgCoords[ptIdx][0];
                double v = mImgCoords[ptIdx][1];

                mat(row, 0) = 0.0;
                mat(row, 1) = 0.0;
                mat(row, 2) = 0.0;
                mat(row, 3) = 0.0;
                mat(row, 4) = -X;
                mat(row, 5) = -Y;
                mat(row, 6) = -Z;
                mat(row, 7) = -1.0;
                mat(row, 8) = v * X;
                mat(row, 9) = v * Y;
                mat(row, 10) = v * Z;
                mat(row, 11) = v;

                ptIdx++;
            }
            else
            {
                // -X -Y -Z, -1, 0,0,0,0, uX, uY, uZ ,u
                double X = mPosMarkers[ptIdx][0];
                double Y = mPosMarkers[ptIdx][1];
                double Z = mPosMarkers[ptIdx][2];
                double u = mImgCoords[ptIdx][0];
                double v = mImgCoords[ptIdx][1];

                mat(row, 0) = -X;
                mat(row, 1) = -Y;
                mat(row, 2) = -Z;
                mat(row, 3) = -1.0;
                mat(row, 4) = 0.0;
                mat(row, 5) = 0.0;
                mat(row, 6) = 0.0;
                mat(row, 7) = 0.0;
                mat(row, 8) = u * X;
                mat(row, 9) = u * Y;
                mat(row, 10) = u * Z;
                mat(row, 11) = u;
            }
        }

        Eigen::BDCSVD<MatrixB> svd(mat, Eigen::ComputeThinU | Eigen::ComputeThinV);

        // A rare failure case due to rank deficiency
        if (svd.rank() != 12)
            return mMatrixProj_OD;

        for (int row = 0; row < 3; row++)
            for (int col = 0; col < 4; col++)
                mMatrixProj_OD.mat(row, col) = svd.matrixV().col(11)[4 * row + col];

        evaluate(mMatrixProj_OD);
        optimise(mMatrixProj_OD);

        return mMatrixProj_OD;
    };

    void optimise(Solution& solution)
    {
        // Gauss-Newton
        // Or Levenberg-Marquardt
    }

    void evaluate(Solution& sol)
    {
        sol.avgReprojError = 0.0;
        for (int idx = 0; idx < mPosMarkers.size(); idx++)
        {
            // convert pt3D into homogeneous form
            Vec4D pt3D_hom = Vec4D::Zero();
            pt3D_hom.block<3, 1>(0, 0) = mPosMarkers[idx].block<3, 1>(0, 0);
            pt3D_hom[3] = 1.0;

            Vec3D pt2D_hat = sol.mat * pt3D_hom;

            // normalise 2D pts
            const auto norm = 1.0 / pt2D_hat[2];
            pt2D_hat[0] *= norm;
            pt2D_hat[1] *= norm;
            pt2D_hat[2] = 1.0;

            // calculate reprojection error
            const double u_res = pt2D_hat[0] - mImgCoords[idx][0];
            const double v_res = pt2D_hat[1] - mImgCoords[idx][1];

            sol.reprojErr[idx] = std::sqrt(u_res * u_res + v_res * v_res);
            sol.avgReprojError += sol.reprojErr[idx];
        }
        sol.avgReprojError = sol.avgReprojError / 30.0;
    }

    void evaluate_RANSAC(Solution& sol, double threshold)
    {
        sol.avgReprojError = 0.0;
        sol.numInliers = 0;
        sol.reprojErr.clear();
        for (int idx = 0; idx < mPosMarkers.size(); idx++)
        {
            // convert pt3D into homogeneous form
            Vec4D pt3D_hom = Vec4D::Zero();
            pt3D_hom.block<3, 1>(0, 0) = mPosMarkers[idx].block<3, 1>(0, 0);
            pt3D_hom[3] = 1.0;

            Vec3D pt2D_hat = sol.mat * pt3D_hom;

            // normalise 2D pts
            const auto norm = 1.0 / pt2D_hat[2];
            pt2D_hat[0] *= norm;
            pt2D_hat[1] *= norm;
            pt2D_hat[2] = 1.0;

            // calculate reprojection error (L2 norm)
            const double u_res = pt2D_hat[0] - mImgCoords[idx][0];
            const double v_res = pt2D_hat[1] - mImgCoords[idx][1];

            const double reprojError = std::sqrt(u_res * u_res + v_res * v_res);
            if (reprojError > threshold) // Only consider the reprojection error of inliers
                continue;

            sol.reprojErr.push_back(reprojError);
            sol.numInliers++;
            sol.avgReprojError += reprojError;
        }

        if (sol.numInliers != 0)
            sol.avgReprojError /= static_cast<double>(sol.numInliers);
    }
};