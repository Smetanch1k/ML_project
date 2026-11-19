using UnityEngine;
using System.Reflection;

public class ApplyComponentsToChildren : MonoBehaviour
{
    [ContextMenu("Apply Components To Children")]
    void Apply()
    {
        Transform[] children = GetComponentsInChildren<Transform>(true);

        foreach (Transform child in children)
        {
            if (child == transform) continue;

            Component[] parentComponents = GetComponents<Component>();

            foreach (Component comp in parentComponents)
            {
                if (comp is Transform) continue;

                System.Type type = comp.GetType();

                Component newComp = child.gameObject.GetComponent(type);

                if (newComp == null)
                    newComp = child.gameObject.AddComponent(type);

                CopyComponentValues(comp, newComp);
            }
        }

        Debug.Log("Components applied to children");
    }

    void CopyComponentValues(Component source, Component destination)
    {
        System.Type type = source.GetType();

        FieldInfo[] fields = type.GetFields(
            BindingFlags.Public |
            BindingFlags.NonPublic |
            BindingFlags.Instance);

        foreach (FieldInfo field in fields)
        {
            field.SetValue(destination, field.GetValue(source));
        }
    }
}
