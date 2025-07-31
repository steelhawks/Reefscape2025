package org.steelhawks.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import org.python.core.*;
import org.python.util.PythonInterpreter;

import java.io.File;
import java.util.Objects;
import java.util.stream.Stream;


/**
 * Utility class for calling Python code via Jython.
 * Scripts must be loaded first using {@link #execFile(String)}.
 *
 * @author Farhan jamil
 */
@SuppressWarnings("unused")
public final class PyBridge {

    private static final PythonInterpreter interpreter = new PythonInterpreter();
    private static final PyBridge INSTANCE = new PyBridge();

    private PyBridge() {}

    public static void init() {
        if (Filesystem.getDeployDirectory().listFiles() != null) {
            for (File f : Objects.requireNonNull(Filesystem.getDeployDirectory().listFiles())) {
                if (f.getName().endsWith(".py")) {
                    getInstance().execFile(f);
                }
            }
        }
    }

    public static synchronized PyBridge getInstance() {
        return INSTANCE;
    }

    /**
     * Loads a Python script into the interpreter (e.g., a .py file).
     * @param path Full path to Python script
     */
    public void execFile(String path) {
        interpreter.execfile(path);
    }

    /**
     * Loads a Python script into the interpreter (e.g., a .py file).
     * @param file File object of Python script
     */
    public void execFile(File file) {
        execFile(file.getPath());
    }

    /**
     * Gets a Python function by name.
     * @param funcName Name of the function in the loaded script
     * @return PyFunction instance or null if not found
     */
    public PyFunction getFunc(String funcName) {
        return interpreter.get(funcName, PyFunction.class);
    }

    /**
     * Calls a Python function with arguments.
     * @param funcName Function name (must be loaded via execfile first)
     * @param args Java arguments to convert and pass
     * @return PyObject result
     */
    public PyObject callFunc(String funcName, Object... args) {
        PyFunction function = getFunc(funcName);
        if (function == null) {
            throw new RuntimeException("Function not found: " + funcName);
        }

        PyObject[] pyArgs = Stream.of(args)
            .map(PyJavaType::wrapJavaObject)
            .toArray(PyObject[]::new);

        return function.__call__(pyArgs);
    }

    /**
     * Gets any Python object (function, class, variable, etc.) by name.
     * @param name Name in Python script
     * @return PyObject
     */
    public PyObject get(String name) {
        return interpreter.get(name);
    }

    /**
     * Instantiates a Python class by name, optionally with constructor args.
     * @param className Name of the class in Python
     * @param args Arguments to pass to constructor
     * @return PyObject instance
     */
    public PyObject createInstance(String className, Object... args) {
        PyObject pyClass = get(className);
        if (pyClass == null) {
            throw new RuntimeException("Class not found: " + className);
        }

        PyObject[] pyArgs = Stream.of(args)
            .map(PyJavaType::wrapJavaObject)
            .toArray(PyObject[]::new);

        return pyClass.__call__(pyArgs);
    }

    /**
     * Calls a method on a Python object.
     * @param obj Python object
     * @param method Method name
     * @param args Arguments to method
     * @return PyObject result
     */
    public PyObject callMethod(PyObject obj, String method, Object... args) {
        PyObject[] pyArgs = Stream.of(args)
            .map(PyJavaType::wrapJavaObject)
            .toArray(PyObject[]::new);

        return obj.invoke(method, pyArgs);
    }

    /**
     * Converts a PyObject to a Java object of a given type.
     */
    public <T> T toJava(PyObject obj, Class<T> type) {
        Object javaObj = obj.__tojava__(type);
        if (type.isInstance(javaObj)) {
            return type.cast(javaObj);
        }
        return null;
    }


    /**
     * Returns the directory of the Python script.
     * <p>
     * You can choose to end {@code pyName} with ".py" or not, either or is acceptable.
     * @param pyName The name of the desired Python script
     * @return The absolute directory
     */
    public File getFileDir(String pyName) {
        boolean endsWithPy = pyName.endsWith(".py");
        try {
            return new File(
                Filesystem.getDeployDirectory().getAbsolutePath() + new File("/py/" + pyName + (endsWithPy ? "" : ".py")));
        } catch (RuntimeException e) {
            DriverStation.reportError("Failed to get file directory.", true);
        }
        return null;
    }
}
