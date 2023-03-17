using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Unit : MonoBehaviour
{
    public Transform target;
    float speed = 23;
    Vector3[] path;
    int targetIndex;

    void Start(){   // unit request path
        PathRequestManager.RequestPath(transform.position, target.position, OnPathFound);
    }

    public void OnPathFound(Vector3[] newPath, bool pathSuccessful) {
        if (pathSuccessful) {
            path = newPath;
            StopCoroutine("FollowPath");
            StartCoroutine("FollowPath");
        }
    }

    IEnumerator FollowPath() {
        Vector3 currentWaypoint = path[0];

        while (true) {
            if (transform.position == currentWaypoint){
                targetIndex ++;   // advance to next waypoint in the path
                if (targetIndex >= path.Length) {
                    yield break;    // exit coroutine
                }
                currentWaypoint = path[targetIndex];
            }
            transform.position = Vector3.MoveTowards(transform.position, currentWaypoint, speed * Time.deltaTime);
            yield return null;    // next frame
        }
    }

    // visualize unit pathing
    public void OnDrawGizmos() {
        if (path != null) {
            for (int i = targetIndex; i < path.Length; i++){
                Gizmos.color = Color.blue;
                Gizmos.DrawCube(path[i], Vector3.one);  // vector3.one (scale)

                if (i == targetIndex) {
                    Gizmos.DrawLine(transform.position, path[i]);
                }
                else {
                    Gizmos.DrawLine(path[i-1], path[i]);
                }


            }
        }
    }

}

