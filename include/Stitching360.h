#pragma once

// #if defined(_DLL_EXPORTS) // inside DLL
// #   define DLL_API   __declspec(dllexport)
// #else // outside DLL
// #   define DLL_API   __declspec(dllimport)
// #endif  // XYZLIBRARY_EXPORT

// #include <opencv2\opencv.hpp>
#include <opencv2/opencv.hpp>
//#include </usr/local/include/opencv4/opencv2/opencv.hpp>
#include <experimental/filesystem>
// #include <filesystem>
#include <fstream>

class SurroundView
{
public:
    /************************相机标定以及矫正****************************/
    virtual int Init(int nSrcHeight, int nSrcWidth) = 0;
    virtual cv::cuda::GpuMat Undistort(cv::cuda::GpuMat &mSrcImg) = 0;

    /************************逆投影变换*******************************/
    virtual cv::Mat PerspectiveTransform(cv::InputArray aInput, cv::Point2f *pSrcPoints, cv::Point2f *pDstPoints, cv::Size sOutputSize, int nOrientation) = 0;

    /*************************图像拼接**********************************/
    virtual cv::Mat ImageStitching(int nWidth, int nHeight, cv::Mat aInputLeft, cv::Mat aInputRight, cv::Mat aInputFront, cv::Mat aInputBack,
        std::vector<cv::Point> vPtsInputLeft, std::vector<cv::Point> vPtsInputRight, std::vector<cv::Point> vPtsInputFront, std::vector<cv::Point> vPtsInputBack) = 0;

};

class Stitching360 :public SurroundView
{
private:
    std::string                             m_sImageRoot;	/* ͼƬ�ļ��� */
    std::string                             m_sLastName;    /* ͼƬ��׺�� */
    std::string                             m_sCaliResult; /* ��궨���ݵ��ļ���*/
    cv::Size                                m_szImage;
    cv::Size                                m_szBoard;	/****    �������ÿ�С��еĽǵ���       ****/
    int                                     m_nImageCount;	/****    �궨ͼ������     ****/
    int                                     m_nSuccessImageNum;                /****   �ɹ���ȡ�ǵ������ͼ����    ****/
    cv::Matx33d                             m_mIntrinsicMatrix;    /*****    ������ڲ�������    ****/
    cv::Matx33d                             m_mNewIntrinsicMat;   /** ����ͷ�µ��ڲ����ڽ��� **/
    cv::Vec4d                               m_vDistortionCoeffs;     /* �������4������ϵ����k1,k2,k3,k4*/
    std::vector<cv::Mat>                    m_vImageSeq;					/* ����ͼ�� */
    std::vector<std::vector<cv::Point2f>>   m_vCornersSeq;    /****  �����⵽�����нǵ�       ****/
    std::vector<cv::Point2f>                n_vCorners;                  /****    ����ÿ��ͼ���ϼ�⵽�Ľǵ�       ****/
    std::vector<cv::Vec3d>                  m_vRotationVectors;                           /* ÿ��ͼ�����ת���� */
    std::vector<cv::Vec3d>                  m_vTranslationVectors;                        /* ÿ��ͼ���ƽ������ */
    cv::cuda::GpuMat                        m_cmMap1; /* ���ս�����ӳ��� */
    cv::cuda::GpuMat                        m_cmMap2; /* ���ս�����ӳ��� */

    int findCorners();
    int cameraCalibrate(int count);
    int savePara();
    void OnMouseAction(int event, int x, int y, int flags, void *para);


public:
    Stitching360();
    ~Stitching360();
    /************************����궨�Լ�����****************************/
    virtual int Init(int nSrcHeight, int nSrcWidth);
    virtual cv::cuda::GpuMat Undistort(cv::cuda::GpuMat &mSrcImg);

    /************************��ͶӰ�任*******************************/
    virtual cv::Mat PerspectiveTransform(cv::InputArray aInput, cv::Point2f *pSrcPoints, cv::Point2f *pDstPoints, cv::Size sOutputSize, int nOrientation);

    /*************************ͼ��ƴ��**********************************/
    virtual cv::Mat ImageStitching(int nWidth, int nHeight, cv::Mat aInputLeft, cv::Mat aInputRight, cv::Mat aInputFront, cv::Mat aInputBack,
        std::vector<cv::Point> vPtsInputLeft, std::vector<cv::Point> vPtsInputRight, std::vector<cv::Point> vPtsInputFront, std::vector<cv::Point> vPtsInputBack);

};

// extern "C" DLL_API SurroundView *GetStitching();
