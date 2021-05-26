// using MathNet.Numerics.LinearAlgebra;
// using MathNet.Numerics.LinearAlgebra.Single;
using System.Collections.Generic;
using System.Linq;

using System.Collections;
using UnityEngine;

public class ImpedanceController : MonoBehaviour
{
    public bool printLogs = false;
    public Transform obstacle = null;
    public float kGain = 1.0f;
    public float mass = 1.0f;
    public float damping = 2.0f;
    public float stiffness = 1.0f;
    public Vector3 correctionLimit = new Vector3(1000f, 1000f, 1000f);
    private Rigidbody rootBody;
    private float runTime = 20.0f;
    private float a = 0.0f;
    private float b = 0.0f;
    private float c = 0.0f;

    // private Matrix<float> PoseImp = null;
    // private Matrix<float> PoseError = null;
    
    private Vector3 ForceExt;
    // private Matrix<float> AN = null;
    // private Matrix<float> BN = null;
    // private Matrix<float> AD = null;
    // private Matrix<float> BD = null;
    private float A11, A12, A21, A22;
    private float B1, B2;
    private Vector3 positionCorrection = Vector3.zero;
    private Vector3 velocityCorrection = Vector3.zero;
    private Vector3 prevPosition, prevVelocity;
    private Vector3 obstaclePrevPosition;

    // Start is called before the first frame update
    void Start()
    {
        this.rootBody = GetComponent<Rigidbody>();
        if (this.rootBody == null) {
            this.rootBody = GetComponentInParent<Rigidbody>();
        }
        
        this.prevPosition = rootBody.transform.localPosition;
        this.prevVelocity = rootBody.velocity;
        this.obstaclePrevPosition = transform.InverseTransformPoint(obstacle.position);
    }

    public (Vector3, Vector3) GetCorrection() {
        var deltaPosition = rootBody.transform.localPosition - this.prevPosition;
        var deltaVelocity = Vector3.zero; // rootBody.velocity - this.prevVelocity;
        var newObstaclePos = transform.InverseTransformPoint(obstacle.position);
        var obstacleV = (newObstaclePos - this.obstaclePrevPosition)/Time.deltaTime;
        this.obstaclePrevPosition = newObstaclePos;
        this.prevPosition = rootBody.transform.localPosition;
        this.prevVelocity = rootBody.velocity;
        return GetCorrection(deltaPosition, deltaVelocity, obstacleV);
    }
    public (Vector3, Vector3) GetCorrection(Vector3 deltaPosition, Vector3 deltaTargetV, Vector3 obstacleV)
    {
        InitState();
        ForceEvaluation(obstacleV);
        // AD = DenseMatrix.Create(2, 2, 0.0f);
        // BD = DenseMatrix.Create(2, 1, 0.0f);
        float D = Mathf.Pow(-a, 2) + 4 * b; // - 4*b
        if (D >= 0)
        {
            float lambda = (a + Mathf.Sqrt(D)) / 2;

            // AD[0, 0] = 1 - lambda * runTime;
            // AD[0, 1] = runTime;
            // AD[1, 0] = -b * runTime;
            // AD[1, 1] = 1 - lambda * runTime - a * runTime;
            // AD = expLambdaT * AD;
            // BD[0, 0] = expLambdaT * (1 - lambda * runTime) - 1;
            // BD[1, 0] = -expLambdaT * b * runTime;
            // BD = -c / b * BD;
            float expLambdaT = Mathf.Exp(lambda * runTime);
            A12 = expLambdaT * (runTime);
            A11 = expLambdaT * (1 - lambda * runTime);
            A21 = expLambdaT * (-b * runTime);
            A22 = expLambdaT * (1 - lambda * runTime - a * runTime);
            B1 = -c / b * (expLambdaT * (1 - lambda * runTime) - 1f);
            B2 = -c / b * (-expLambdaT * b * runTime);
            //PoseImp = AD * PoseError + BD * ForceExt;
            // Matrix<float> AxisErrorX = DenseMatrix.Create(2, 1, 0.0f);
            // Matrix<float> AxisErrorY = DenseMatrix.Create(2, 1, 0.0f);
            // Matrix<float> AxisErrorZ = DenseMatrix.Create(2, 1, 0.0f);

            float posCorrectionX = A11 * deltaPosition.x + A12 * deltaTargetV.x + B1 * ForceExt.x;
            float velCorrectionX = A21 * deltaPosition.x + A22 * deltaTargetV.x + B2 * ForceExt.x;
            // AxisErrorX[0, 0] = deltaPosition.x;
            // AxisErrorX[1, 0] = deltaTargetV.x;
            // AxisErrorX = AD * AxisErrorX + BD * ForceExt.x;

            float posCorrectionY = A11 * deltaPosition.y + A12 * deltaTargetV.y + B1 * ForceExt.y;
            float velCorrectionY = A21 * deltaPosition.y + A22 * deltaTargetV.y + B2 * ForceExt.y;
            // AxisErrorY[0, 0] = deltaPosition.y;
            // AxisErrorY[1, 0] = deltaTargetV.y;
            // AxisErrorY = AD * AxisErrorY + BD * ForceExt.y;

            float posCorrectionZ = A11 * deltaPosition.z + A12 * deltaTargetV.z + B1 * ForceExt.z;
            float velCorrectionZ = A21 * deltaPosition.z + A22 * deltaTargetV.z + B2 * ForceExt.z;
            // AxisErrorZ[0, 0] = deltaPosition.z;
            // AxisErrorZ[1, 0] = deltaTargetV.z;
            // AxisErrorZ = AD * AxisErrorZ + BD * ForceExt.z;

            if (printLogs) {
                // print($"{AD[0,0]} {AD[0,1]} {AD[1,0]} {AD[1,1]}");
                // print($"{AxisErrorX[0,0]}, {AxisErrorY[0,0]}, {AxisErrorZ[0,0]}");
            }
            positionCorrection = new Vector3(
                Mathf.Clamp(posCorrectionX, -correctionLimit.x, correctionLimit.x),
                Mathf.Clamp(posCorrectionY, -correctionLimit.y, correctionLimit.y),
                Mathf.Clamp(posCorrectionZ, -correctionLimit.z, correctionLimit.z));
            velocityCorrection = new Vector3(velCorrectionX, velCorrectionY, velCorrectionZ);

            return (positionCorrection, velocityCorrection);
        }
        Debug.Log("Problems");
        return (new Vector3(0.0f, 0.0f, 0.0f), new Vector3(0.0f, 0.0f, 0.0f));
    }

    void InitState()
    {
        a = -damping / mass;
        b = -stiffness / mass;
        c = 1 / mass;
        runTime = Time.deltaTime;
    }

    void ForceEvaluation(Vector3 obsV)
    {
        ForceExt = obsV * kGain;
    }

}
